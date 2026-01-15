from behaviors.base import Behavior, BehaviorStatus

class ReturnToBase(Behavior):
    def start(self, **_):
        pass

    def update(self, **_):
        return BehaviorStatus.SUCCEEDED
