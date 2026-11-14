# Test code, not part of the library: a MotorGroup that arms on the arming window alone, for bench
# scripts that run bidirectional motors with no ESC attached (nothing will ever reply on those pins).
# Never use it where an ESC is attached: arming then skips the check that the ESC is listening.

from motor_group import MotorGroup


class GroupWithoutReplyGate(MotorGroup):
    def bidir_ready(self, now):
        return True
