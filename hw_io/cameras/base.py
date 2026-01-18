# hw_io/cameras/base.py

from abc import ABC, abstractmethod

class Camera(ABC):
    @abstractmethod
    def see(self):
        """Return detected markers"""
        raise NotImplementedError

    @abstractmethod
    def capture(self):
        """Return image/frame if supported"""
        raise NotImplementedError
