import threading
from typing import Generic, Type, TypeVar

from unitree_sdk2py.core.channel import ChannelSubscriber

T = TypeVar("T")


class G1Subscriber(Generic[T]):
    def __init__(self, topic: str, t: Type[T], auto_start: bool = True):
        self.sub = ChannelSubscriber(topic, t)
        self.l = threading.Lock()
        self.msg: T | None = None
        self.ev = threading.Event()
        if auto_start:
            self.start()

    def __set_msg(self, msg: T):
        with self.l:
            self.msg = msg
            self.ev.set()

    def get_msg(self) -> T:
        self.ev.wait()
        with self.l:
            msg = self.msg
        assert msg is not None
        return msg

    def get_msg_nowait(self) -> T | None:
        with self.l:
            msg = self.msg
        return msg

    def start(self):
        self.ev.clear()
        self.sub.Init(handler=self.__set_msg)

    def stop(self):
        self.sub.Close()
        with self.l:
            self.msg = None
