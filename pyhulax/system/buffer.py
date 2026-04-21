import threading


class Buffer(threading.Thread):

    def __init__(self):
        self._buf = []
        self._blocking = True
        self._cond = threading.Condition()
        self._timeout = None

    def set_data(self, data):
        """
        Set data into the buffer
        """
        if self._blocking:
            self._cond.acquire()
            self._buf += data
            self._cond.notify()
            self._cond.release()
        else:
            self._buf += data

    def get_data(self, size):
        """
        Get data from the buffer
        """
        if self._blocking:
            self._cond.acquire()
            while True:
                if len(self._buf) >= size:
                    tmp = self._buf[0:size]
                    self._buf = self._buf[size:]
                    self._cond.release()
                    return tmp
                else:
                    if not self._cond.wait(self._timeout):
                        return None
        elif not self._blocking:
            if len(self._buf) >= size:
                tmp = self._buf[0:size]
                self._buf = self._buf[size:]
                return tmp
            else:
                return None

    def get_data_try(self, size):
        """
        Try to get data from the buffer, you should use get_data_confirm to make sure
        """
        if self._blocking:
            while True:
                if len(self._buf) >= size:
                    self._try_data = size
                    return self._buf[0:size]
                else:
                    self._cond.acquire()
                    if not self._cond.wait(self._timeout):
                        return None
                    self._cond.release()
        elif not self._blocking:
            if len(self._buf) >= size:
                self._try_data = size
                return self._buf[0:size]
            else:
                return None

    def get_data_confirm(self):
        if self._blocking:
            self._cond.acquire()
            self._buf = self._buf[self._try_data :]
            self._cond.release()
        else:
            self._buf = self._buf[self._try_data :]
        self._try_data = 0

    def wake_up(self):
        self._cond.acquire()
        self._cond.notify()
        self._cond.release()

    def clean_buf(self):
        """
        Clean up the buffer
        """
        if self._blocking:
            self._cond.acquire()
            self._buf.clear()
            self._cond.release()
        else:
            self._buf.clear()

    def set_blocking(self, mode):
        """
        Set blocking mode to blocking or non-blocking
        """
        if mode == 1:
            self._blocking = True
        elif mode == 0:
            self._blocking = False

    def set_timeout(self, time):
        if isinstance(time, int) or isinstance(time, float):
            if time >= 0:
                self._timeout = time
        elif time is None:
            self._timeout = None

    def __len__(self):
        return len(self._buf)


if __name__ == "__main__":
    buf = Buffer()
    buf.set_timeout(2)
    buf.set_data(b"123456789")
    print(buf.get_data_try(2))
    print(buf.get_data_try(2))
    buf.get_data_confirm()
    print(buf.get_data_try(2))
    print(buf.get_data_try(2))
    """
      buf = Buffer()
    """  #! MISSING: Line (124+)
