import threading


class Slots:
    def __init__(self, slot_id, timeout=None):
        self._id = slot_id
        self._cond = threading.Condition()
        self._num = 0
        self._timeout = timeout
        self._data = None


class Event:
    def __init__(self):
        self._slot_map = {}
        self._signal_map = {}

    def wait_for_signal(self, slot_id, timeout=None):
        if not (
                ((isinstance(timeout, int) or isinstance(timeout, float)) and timeout >= 0)
                or timeout is None
        ):
            print("set timeout error")
            return
        if slot_id not in self._slot_map:
            self._slot_map[slot_id] = Slots(slot_id)
            self._slot_map[slot_id]._timeout = timeout

        slot = self._slot_map[slot_id]
        with slot._cond:
            slot._num += 1
            if not slot._cond.wait(timeout=slot._timeout):
                slot._num -= 1
                if slot._num == 0:
                    del self._slot_map[slot_id]
                return None
            data = slot._data
            slot._num -= 1
            if slot._num == 0:
                del self._slot_map[slot_id]

        return data
        # if id not in self._slot_map:
        #     self._slot_map[id] = Slots(id)

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    # return _data

    def create_signal(self, signal_id, data):
        # if id in self._slot_map:
        #     self._slot_map[id]._data = data
        
        
        #     # print('notify')
        #     self._slot_map[id]._cond.release()
        if signal_id in self._slot_map:
            slot = self._slot_map[signal_id]
            with slot._cond:
                slot._data = data
                slot._cond.notify_all()

    def reset_timeout(self, slot_id, timeout):
        # if not (((isinstance(timeout, int) or isinstance(timeout, float)) and timeout >= 0) or timeout == None):
        #     return
        # self._slot_map[id].timeout = timeout
        if not (
            ((isinstance(timeout, int) or isinstance(timeout, float)) and timeout >= 0)
            or timeout is None
        ):
            return
        if slot_id in self._slot_map:
            self._slot_map[slot_id]._timeout = timeout


if __name__ == "__main__":

    import time
    import state

    _event = Event()

    def thread1():
        time.sleep(1)
        print(_event.wait_for_signal(state.SysState.P_State_GetHeartbeat, 2))

    def thread2():
        print(_event.wait_for_signal(state.SysState.P_State_GetHeartbeat, 2))

    _thread = threading.Thread(target=thread1, name="thread1")
    _thread.setDaemon(True)
    _thread.start()

    _thread = threading.Thread(target=thread2, name="thread2")
    _thread.setDaemon(True)
    _thread.start()

    time.sleep(1)
    _event.create_signal(state.SysState.P_State_GetHeartbeat, 2222)

    while True:
        pass
