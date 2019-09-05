# Spawns two threads corresponding to Fall Detection and other features.
import threading
from glove import main
from MPU9150 import start

threading.Thread(target=main).start()
threading.Thread(target=start).start()
