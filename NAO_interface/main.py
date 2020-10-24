""" Execute all needed processes as concurrent subprocesses. """

import multiprocessing
import time

import camera_extractor
import commands_executioner
import events_extractor
import memory_extractor


# array of processes to be executed
# specify the function name
proc_names = [camera_extractor.main,
              events_extractor.main,
              memory_extractor.main,
              commands_executioner.main]
# for the simulation we only need this process
proc_names = [commands_executioner.main]


if __name__ == "__main__":
    # running processes
    processes = []

    # start all the processes
    for name in proc_names:
        proc = multiprocessing.Process(target=name)
        proc.daemon = True
        proc.start()
        processes.append(proc)

    # eternal loop of execution
    try:
        while True:
            time.sleep(30)
    except KeyboardInterrupt:
        for proc in processes:
            # wait until all the subprocess exit before finish this main process
            proc.join()
        print("exit main")
