import os
import tempfile

APP_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(APP_DIR, 'assets')

RESULTS_DIR = os.path.join(tempfile.gettempdir(), 'results')
os.makedirs(RESULTS_DIR, exist_ok=True)

def result_path(filename):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    return os.path.join(RESULTS_DIR, filename)

