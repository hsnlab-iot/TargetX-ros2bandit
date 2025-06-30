from multiprocessing import Manager, Lock

def get_shared_resources():
    manager = Manager()
    shared_data = manager.dict()
    lock = manager.Lock()
    return shared_data, lock
