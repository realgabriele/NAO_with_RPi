"""execute all needed processes
    as concurrent subprocesses"""

import multiprocessing

import camera_extractor
import commands_executioner
import events_extractor
import memory_extractor

proc_names = [camera_extractor.main,
              commands_executioner.main,
              events_extractor.main,
              memory_extractor.main]

if __name__ == "__main__":
    processes = []

    for name in proc_names:
        proc = multiprocessing.Process(target=name)
        proc.daemon = True
        proc.start()
        processes.append(proc)

    try:
        while True:
            pass
    except KeyboardInterrupt:
        for proc in processes:
            proc.join()
        print("exit main")
