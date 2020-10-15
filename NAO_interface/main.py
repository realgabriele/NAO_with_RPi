"""execute all needed processes
    as concurrent subprocesses"""

import multiprocessing

import camera_extractor
import commands_executioner
import events_extractor
import memory_extractor

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
            pass
    except KeyboardInterrupt:
        print("exit main")
