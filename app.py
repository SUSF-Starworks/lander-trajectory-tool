import base64
import json
import os
import time

from nicegui import app, run, ui

from vtvl_sim import(
    CONTROLLER_REGISTRY,
    animate_descent, plot_engine, plot_state, plot_trajectory,
    compute_engine_metrics, compute_state_metrics,
    compute_trajectory_metrics,
    build_setup, sim_run,
) # Custom vtvl sim library that can be found on my github
from paths import APP_DIR, ASSETS_DIR, RESULTS_DIR, result_path

# Creating the directory that will carry results
app.add_static_files('/results', RESULTS_DIR)

# Importing the Starworks logo into the app
_LOGO_FILE = os.path.join(ASSETS_DIR, 'logo.png')
with open(_LOGO_FILE, 'rb') as _f:
    LOGO_URI = 'data:image/png;base64,' + base64.b64encode(_f.read()).decode()

ui.add_head_html('''
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet"
      href="https://fonts.googleapis.com/css2?family=Source+Sans+3:wght@400;600;700&display=swap">
<style>
  body, .nicegui-content, .nicegui-content * {
    font-family: 'Source Sans Pro', 'Source Sans 3', sans-serif;
  }
  .material-icons, .q-icon {
    font-family: 'Material Icons' !important;
  }
  body.body--dark { background: #000000 !important; }
  body:not(.body--dark) { background: #FFFFFF !important; }
</style>
''')

ui.colors(primary='#01788D', accent='#01788D', dark='#000000')
dark_mode = ui.dark_mode(True)

# Title row sitting at the top of the page w/ dark mode toggle and logo
with ui.row().classes('w-full items-center justify-end relative'):

    ui.label('Lander Trajectory Analysis').classes('text-2xl absolute left-1/2 -translate-x-1/2').style(
        'font-size: 2.5rem; font-weight: 700')
    ui.html(f'<img src="{LOGO_URI}" alt="Starworks SUSF 25-26" '
            f'style="height: 7.5rem; width: auto;">').classes('mr-auto')
    ui.button(icon='dark_mode', on_click=dark_mode.toggle).props('flat round').tooltip('Toggle dark mode')

# Loads up the default values from a .json scenario file
with open(os.path.join(APP_DIR, 'default.json')) as _f:
    _DEFAULTS = json.load(_f)
_DEFAULT_SIM = _DEFAULTS['sim_setup']

DEFAULT_PARAMS = _DEFAULT_SIM['params']
DEFAULT_INITIAL_STATE = _DEFAULT_SIM['initial_state']
DEFAULT_LANDING_TOLERANCE = _DEFAULT_SIM['landing_tolerance']
DEFAULT_SOLVER_SETUP = _DEFAULTS['solver_setup']
DEFAULT_CONTROLLER = _DEFAULT_SIM['controller_name']
DEFAULT_PHASES = _DEFAULT_SIM['phases']
DEFAULT_T_END = DEFAULT_PHASES[0]['t_end'] if DEFAULT_PHASES else 30.0

# Controller whose target is a pitch angle rather than an x/z waypoint; its
# gimbal-only inner loop holds hover thrust and does not land. The GUI shows a
# θ-target field only for it.
ATTITUDE_DEMO = 'Attitude PD (inner-loop demo)'

# (key, label) for the widgets that let the user override the physics params and
# initial state. Keys match the scenario JSON so loading/saving is a direct copy.
PARAM_FIELDS = [
    ('m', 'm — mass [kg]'),
    ('I', 'I — inertia [kg·m²]'),
    ('L', 'L — moment arm [m]'),
    ('g', 'g [m/s²]'),
    ('T_max', 'T_max [N]'),
    ('T_min', 'T_min [N]'),
    ('isp', 'Isp [s]'),
    ('delta_max_deg', 'δ_max [deg]'),
    ('tilt_limit_deg', 'tilt limit [deg]'),
]
INITIAL_STATE_FIELDS = [
    ('x', 'x [m]'), ('z', 'z [m]'), ('xdot', 'ẋ [m/s]'),
    ('zdot', 'ż [m/s]'), ('theta', 'θ [rad]'), ('thetadot', 'θ̇ [rad/s]'),
]
SOLVER_METHODS = ['RK45', 'RK23', 'DOP853', 'Radau', 'BDF', 'LSODA']

# Below inputs are populated upon running the sim
param_inputs = {}
initial_state_inputs = {}
gain_inputs = {}
solver_inputs = {}
misc_inputs = {}  # landing_tolerance

phase_rows = []

def add_phase_row(x_target=0.0, z_target=0.0, t_end=DEFAULT_T_END):
    """
    adds a new row to the phase_rows list
    :param x_target:
    :param z_target:
    :param t_end:
    :return:
    """
    with phases_container:
        with ui.row() as row:
            with ui.row():
                with ui.column():
                    x_input = ui.number(label='x_target [m]', value=x_target)
                with ui.column():
                    z_input = ui.number(label='z_target [m]', value=z_target)
                with ui.column():
                    t_end_input = ui.number(label='t_end [s]', value=t_end)
            entry = {'row': row, 'x_target': x_input, 'z_target': z_input, 't_end': t_end_input}
            ui.button(icon='delete', on_click=lambda: remove_phase_row(entry)).props('flat dense')
    phase_rows.append(entry)

def remove_phase_row(entry):
    """
    self-explanatory when paired with the add_phase_row function
    :param entry:
    :return:
    """
    entry['row'].delete()
    phase_rows.remove(entry)

def rebuild_gains(controller_name, values=None):
    """Render the gain inputs for `controller_name`, seeded from `values` (a gains
    dict, e.g. from a loaded scenario) or the controller's registry defaults.

    Each controller declares its own gain_fields, so the panel is rebuilt from
    scratch on every controller change rather than assuming the cascade's six."""
    entry = CONTROLLER_REGISTRY[controller_name]
    defaults = entry['defaults']
    seed = values or defaults
    gain_inputs.clear()
    gains_container.clear()
    with gains_container:
        with ui.row().classes('flex-wrap'):
            for field in entry['gain_fields']:
                gain_inputs[field] = ui.number(
                    label=field, value=seed.get(field, defaults[field]), format='%.3f'
                )

def on_controller_change():
    """Sync the gain panel and the θ-target field to the selected controller."""
    name = controller_select.value
    rebuild_gains(name)
    theta_target_input.visible = (name == ATTITUDE_DEMO)

def _placeholder_box(text):
    """Creates placeholder boxses, for plot locations when plots have not been generated yet
    :param text: """
    with ui.element('div').classes(
        'w-64 h-48 border-2 border-dashed border-gray-400 rounded '
        'flex items-center justify-center'
    ):
        ui.label(text).classes('text-gray-400 text-center px-2')

def show_placeholders():
    for panel, text in (
        (state_panel, 'State plot will appear here'),
        (traj_panel, 'Trajectory plot will appear here'),
        (engine_panel, 'Engine plot will appear here'),
        (anim_panel, 'Animation will appear here (if enabled)'),
    ):
        panel.clear()
        with panel:
            _placeholder_box(text)

def _metric_tile(label, value):
    with ui.column().classes('items-center px-4'):
        ui.label(value).classes('text-xl font-bold')
        ui.label(label).classes('text-xs text-grey-6')

# (key, symbol, unit, decimals) for the final-state table. theta/thetadot arrive
# already converted to degrees by compute_state_metrics.
FINAL_STATE_ROWS = [
    ('x', 'x', 'm', 2),
    ('z', 'z', 'm', 2),
    ('xdot', 'ẋ', 'm/s', 2),
    ('zdot', 'ż', 'm/s', 2),
    ('theta', 'θ', 'deg', 1),
    ('thetadot', 'θ̇', 'deg/s', 1),
]

def _show_state_metrics(metrics):
    with ui.row().classes('w-full justify-center gap-6'):
        _metric_tile('Max speed', f"{metrics['max_speed']:.2f} m/s")
        _metric_tile('Max angular rate', f"{metrics['max_rate_deg']:.2f} °/s")
        _metric_tile('Max attitude angle', f"{metrics['max_attitude_deg']:.1f}°")

    final_state = metrics['final_state']
    with ui.column().classes('w-full items-center pt-2'):
        ui.label('Final state vector').classes('text-xs text-grey-6')
        ui.table(
            columns=[
                {'name': 'component', 'label': '', 'field': 'component', 'align': 'left'},
                {'name': 'value', 'label': 'Value', 'field': 'value', 'align': 'right'},
                {'name': 'unit', 'label': 'Unit', 'field': 'unit', 'align': 'left'},
            ],
            rows=[
                {
                    'component': symbol,
                    'value': f'{final_state[key]:.{decimals}f}',
                    'unit': unit,
                }
                for key, symbol, unit, decimals in FINAL_STATE_ROWS
            ],
            row_key='component',
        ).props('dense flat').classes('w-64')

def _show_trajectory_metrics(metrics):
    with ui.row().classes('w-full justify-center gap-6'):
        if metrics['landed']:
            _metric_tile('Flight time', f"{metrics['touchdown_time']:.2f} s")
            _metric_tile('Apogee', f"{metrics['apogee']:.1f} m")
            _metric_tile('Lateral touchdown error', f"{metrics['touchdown_error']:.2f} m")
            _metric_tile('Touchdown speed', f"{metrics['vel_touchdown']:.2f} m/s")
            _metric_tile('Touchdown angle', f"{metrics['angle_touchdown_deg']:.1f}°")
        else:
            _metric_tile('Status', 'No touchdown')
            _metric_tile('Apogee', f"{metrics['apogee']:.1f} m")
            _metric_tile('Final altitude error', f"{metrics['altitude_error']:.2f} m")
            _metric_tile('Final x error', f"{metrics['touchdown_error']:.2f} m")
        _metric_tile('Impulse', f"{metrics['total_impulse']:.0f} Ns")
        _metric_tile('Propellant mass', f"{metrics['propellant_mass']:.2f} kg")
        _metric_tile('Propellant — ascent', f"{metrics['propellant_ascent']:.2f} kg")
        _metric_tile('Propellant — descent', f"{metrics['propellant_descent']:.2f} kg")

def _show_engine_metrics(metrics):
    with ui.row().classes('w-full justify-center gap-6'):
        _metric_tile('Max throttle', f"{metrics['throttle_max']:.1f} %")
        _metric_tile('Min throttle', f"{metrics['throttle_min']:.1f} %")
        _metric_tile('Average throttle', f"{metrics['throttle_avg']:.1f} %")

def apply_scenario(raw):
    """Populate every input widget from a parsed scenario dict. Accepts either a
    full solver scenario file ({'sim_setup': ..., 'solver_setup': ..., 'outputs': ...})
    or a bare sim_setup dict. Missing keys leave the current widget value untouched."""
    sim = raw.get('sim_setup', raw)
    solver = raw.get('solver_setup', {})
    outputs = raw.get('outputs', {})

    params = sim.get('params', {})
    for key, inp in param_inputs.items():
        if key in params:
            inp.value = params[key]

    # Rebuild the gain panel for the loaded controller before seeding values, so
    # the widgets on screen match that controller's gain_fields (not whichever
    # controller was selected before). Then sync the θ-target visibility.
    controller = sim.get('controller_name')
    if controller in CONTROLLER_REGISTRY:
        controller_select.value = controller
        rebuild_gains(controller, sim.get('gains'))
        theta_target_input.visible = (controller == ATTITUDE_DEMO)
    else:
        gains = sim.get('gains', {})
        for key, inp in gain_inputs.items():
            if key in gains:
                inp.value = gains[key]

    if 'phases' in sim:
        for entry in list(phase_rows):
            remove_phase_row(entry)
        for p in sim['phases']:
            add_phase_row(
                p.get('x_target', 0.0), p.get('z_target', 0.0), p.get('t_end', DEFAULT_T_END)
            )
        # θ-target is per-phase in the schema but a single GUI field; take it from
        # the first phase that carries a non-zero one (the demo uses one phase).
        for p in sim['phases']:
            if p.get('theta_target_deg'):
                theta_target_input.value = p['theta_target_deg']
                break

    state = sim.get('initial_state', {})
    for key, inp in initial_state_inputs.items():
        if key in state:
            inp.value = state[key]

    if 'landing_tolerance' in sim:
        misc_inputs['landing_tolerance'].value = sim['landing_tolerance']

    if 'max_step' in solver:
        solver_inputs['max_step'].value = solver['max_step']
    if 'method' in solver:
        solver_inputs['method'].value = solver['method']

    if 'animation' in outputs:
        animation_checkbox.value = bool(outputs['animation'])

    ui.notify('Scenario loaded', type='positive')

async def on_upload(e):
    try:
        data = await e.file.read()  # NiceGUI 3.x: FileUpload.read() is async and returns bytes
        raw = json.loads(data.decode('utf-8'))
    except (json.JSONDecodeError, UnicodeDecodeError) as err:
        ui.notify(f'Could not read {e.file.name}: {err}', type='negative', multi_line=True)
        return
    try:
        apply_scenario(raw)
    except Exception as err:  # noqa: BLE001 — surface any malformed-scenario error to the user
        ui.notify(f'Could not load scenario: {err}', type='negative', multi_line=True)

def collect_sim_setup():
    """Read the current widgets into a sim_setup dict. Params keep degrees
    (delta_max_deg/tilt_limit_deg), the exact same shape the scenario JSON uses."""
    # θ-target only applies to the attitude-hold demo; the mission controllers
    # get 0 (they ignore it, and the schema defaults it to 0 anyway).
    theta_target_deg = theta_target_input.value if controller_select.value == ATTITUDE_DEMO else 0.0
    return {
        'params': {key: inp.value for key, inp in param_inputs.items()},
        'controller_name': controller_select.value,
        'gains': {field: inp.value for field, inp in gain_inputs.items()},
        'phases': [
            {
                'x_target': entry['x_target'].value,
                'z_target': entry['z_target'].value,
                't_end': entry['t_end'].value,
                'theta_target_deg': theta_target_deg,
            }
            for entry in phase_rows
        ],
        'initial_state': {key: inp.value for key, inp in initial_state_inputs.items()},
        'landing_tolerance': misc_inputs['landing_tolerance'].value,
    }

def collect_solver_setup():
    return {
        'max_step': solver_inputs['max_step'].value,
        'method': solver_inputs['method'].value,
    }

def on_download():
    scenario = {
        'sim_setup': collect_sim_setup(),
        'solver_setup': collect_solver_setup(),
        # Only animation has a GUI toggle; the rest default on so the saved file
        # runs fully through the solver's run_scenarios pipeline.
        'outputs': {
            'trajectory': 1,
            'state': 1,
            'animation': int(animation_checkbox.value),
            'report': 1,
            'csv': 1,
            'engine': 1,
        },
    }
    name = (scenario_name_input.value or '').strip() or 'scenario'
    # Keep it a safe single filename: drop path separators, ensure .json extension.
    name = name.replace('/', '_').replace('\\', '_')
    if not name.lower().endswith('.json'):
        name += '.json'
    ui.download.content(json.dumps(scenario, indent=2), name, 'application/json')

async def on_run_click():
    raw_sim_setup = collect_sim_setup()
    raw_solver_setup = collect_solver_setup()

    try:
        sim_setup, solver_setup = build_setup(raw_sim_setup, raw_solver_setup)
    except ValidationError as e:
        ui.notify(str(e), type='negative', multi_line=True)
        return

    spinner.visible = True
    try:
        sim_results = await run.io_bound(sim_run, sim_setup, solver_setup)

        traj_fig = plot_trajectory(sim_results)
        traj_fig.savefig(result_path('last_gui_trajectory.png'), dpi=300)

        state_fig = plot_state(sim_results, sim_setup)
        state_fig.savefig(result_path('last_gui_state.png'), dpi=300)

        engine_fig = plot_engine(sim_results, sim_setup)
        engine_fig.savefig(result_path('last_gui_engine.png'), dpi=300)

        state_metrics = compute_state_metrics(sim_results)
        traj_metrics = compute_trajectory_metrics(sim_setup, sim_results)
        engine_metrics = compute_engine_metrics(sim_setup, sim_results)

        v = time.time()  # cache-buster: the filenames are stable across runs
        for panel, name, width, show_metrics, metrics in (
            (state_panel, 'last_gui_state.png', 'w-[60em]', _show_state_metrics, state_metrics),
            (traj_panel, 'last_gui_trajectory.png', 'w-96', _show_trajectory_metrics, traj_metrics),
            (engine_panel, 'last_gui_engine.png', 'w-[60em]', _show_engine_metrics, engine_metrics),
        ):
            panel.clear()
            with panel:
                ui.image(f'/results/{name}?v={v}').classes(f'{width} max-w-full')
                show_metrics(metrics)

        anim_panel.clear()
        with anim_panel:
            if not animation_checkbox.value:
                _placeholder_box('Animation not generated — enable "Generate animation" and Run again')
            else:
                # io_bound yields None if the app is shutting down mid-encode.
                rendered = await run.io_bound(
                    animate_descent, sim_results, sim_setup,
                    save_path=result_path('last_gui_animation.mp4'),
                )
                # animate_descent reports the file it actually wrote — without
                # ffmpeg that is a GIF, not the .mp4 we asked for.
                anim_out = rendered[2] if rendered else None
                if anim_out is None:
                    _placeholder_box('Animation render was interrupted')
                else:
                    src = f'/results/{os.path.basename(anim_out)}?v={v}'
                    # Height-driven with width: auto — the frame is portrait, so
                    # sizing off the viewport height (rather than the panel width)
                    # is what makes it fill the page instead of letterboxing.
                    size = 'height: 82vh; width: auto; max-width: 100%; object-fit: contain;'
                    if anim_out.lower().endswith('.mp4'):
                        ui.video(src).props('controls').style(size)
                    else:
                        ui.image(src).style(size)
            # Outside the if/else: the animation is a trajectory view, so it
            # carries the same numbers whether or not the video was rendered.
            _show_trajectory_metrics(traj_metrics)
    finally:
        spinner.visible = False

with ui.column().classes('w-full items-center'):
    with ui.tabs().classes('w-full') as result_tabs:
        tab_state = ui.tab('State', icon='show_chart')
        tab_traj = ui.tab('Trajectory', icon='timeline')
        tab_engine = ui.tab('Engine', icon='local_fire_department')
        tab_anim = ui.tab('Animation', icon='movie')
    with ui.tab_panels(result_tabs, value=tab_state).classes('w-full'):
        with ui.tab_panel(tab_state):
            state_panel = ui.column().classes('w-full items-center')
        with ui.tab_panel(tab_traj):
            traj_panel = ui.column().classes('w-full items-center')
        with ui.tab_panel(tab_engine):
            engine_panel = ui.column().classes('w-full items-center')
        with ui.tab_panel(tab_anim):
            anim_panel = ui.column().classes('w-full items-center')

show_placeholders()

ui.separator()

with ui.row().classes('w-full items-center'):
    ui.label('Load scenario (.json)').classes('text-sm text-grey-7')
    ui.upload(on_upload=on_upload, auto_upload=True, label='Scenario file') \
        .props('accept=.json flat dense') \
        .tooltip('Load a solver-format scenario JSON to fill in all fields below')

with ui.row():
    with ui.card():
        ui.label('Controller').classes('text-lg')
        controller_select = ui.select(
            list(CONTROLLER_REGISTRY.keys()),
            value=DEFAULT_CONTROLLER,
            label='Controller',
            on_change=lambda e: on_controller_change(),
        )
        with ui.expansion('Gains', icon='tune', value=True).classes('w-full'):
            gains_container = ui.column()
        # Pitch reference for the attitude-hold demo; the mission controllers take
        # their targets from the x/z waypoints in the Phases card, so it is hidden
        # for them (visibility set by on_controller_change / the initial seed).
        theta_target_input = ui.number(label='θ target [deg]', value=0.0, format='%.2f') \
            .tooltip('Commanded pitch for the Attitude PD demo (applied to all phases)')

    with ui.card():
        ui.label('Phases').classes('text-lg')
        phases_container = ui.column()
        ui.button('+ Add phase', icon='add', on_click=lambda: add_phase_row()).props('flat')

with ui.row():
    with ui.card():
        ui.label('Vehicle parameters').classes('text-lg')
        with ui.grid(columns=2):
            for key, label in PARAM_FIELDS:
                param_inputs[key] = ui.number(label=label, value=DEFAULT_PARAMS[key])

    with ui.card():
        ui.label('Initial state').classes('text-lg')
        with ui.grid(columns=2):
            for key, label in INITIAL_STATE_FIELDS:
                initial_state_inputs[key] = ui.number(label=label, value=DEFAULT_INITIAL_STATE[key])

    with ui.card():
        ui.label('Solver & tolerance').classes('text-lg')
        misc_inputs['landing_tolerance'] = ui.number(
            label='landing tolerance [m]', value=DEFAULT_LANDING_TOLERANCE, format='%.3f'
        )
        solver_inputs['max_step'] = ui.number(
            label='max_step [s]', value=DEFAULT_SOLVER_SETUP['max_step'], format='%.4f'
        )
        solver_inputs['method'] = ui.select(
            SOLVER_METHODS, value=DEFAULT_SOLVER_SETUP['method'], label='method'
        )

# Seed the widgets from the default scenario: gains for the default controller
# (from the JSON, falling back to registry defaults), the θ-target visibility,
# and one phase row per default phase.
rebuild_gains(DEFAULT_CONTROLLER, _DEFAULT_SIM.get('gains'))
theta_target_input.visible = (DEFAULT_CONTROLLER == ATTITUDE_DEMO)
for _phase in DEFAULT_PHASES:
    add_phase_row(_phase.get('x_target', 0.0), _phase.get('z_target', 0.0),
                  _phase.get('t_end', DEFAULT_T_END))

with ui.row().classes('items-center'):
    animation_checkbox = ui.checkbox('Generate animation', value=False)
    ui.button('Run', on_click=on_run_click)
    scenario_name_input = ui.input(label='File name', placeholder='scenario') \
        .props('dense').tooltip('Name for the downloaded scenario file (.json added automatically)')
    ui.button('Save scenario', icon='download', on_click=on_download).props('outline') \
        .tooltip('Download the current fields as a solver-format scenario JSON')
    spinner = ui.spinner(size='lg')
    spinner.visible = False

# Native mode renders the app in a pywebview desktop window (Edge WebView2 on
# Windows) instead of a browser tab; the uvicorn server still runs underneath on
# localhost. Set VTVL_NATIVE=0 to fall back to the browser.
NATIVE = os.getenv('VTVL_NATIVE', '1') != '0'

if NATIVE:
    # A webview has no browser download UI, so ui.download (Save scenario) is a
    # no-op unless pywebview is told to handle downloads itself.
    app.native.settings['ALLOW_DOWNLOADS'] = True
    app.native.window_args['min_size'] = (1100, 700)

ui.run(
    title='Lander Trajectory Tool',
    native=NATIVE,
    window_size=(1600, 1000) if NATIVE else None,
    # The webview runs in its own process; auto-reload re-spawns it and can leave
    # orphaned windows, so it is only enabled for the browser path.
    reload=not NATIVE,
)
