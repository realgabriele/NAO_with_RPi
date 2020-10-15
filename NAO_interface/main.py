"""execute all needed processes"""

import multiprocessing
import camera_extractor, commands_executioner, events_extractor, memory_extractor
import time

processes = [camera_extractor.main,
             commands_executioner.main,
             events_extractor.main,
             memory_extractor.main]

if __name__ == "__main__":
    for proc in processes:
        p = multiprocessing.Process(target=proc)
        p.daemon = True
        p.start()

    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        print("exit main")
