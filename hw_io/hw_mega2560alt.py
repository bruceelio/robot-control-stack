# hw_io/hw_mega2560alt.py

from hw_io.hw_mega2560 import Mega2560IO


class Mega2560AltIO(Mega2560IO):
    """
    Alternate Mega2560 hardware configuration.

    Front drive remains the normal paired DRIVE front command,
    but the Arduino routes the two outputs to the MDD20A
    shooter/collector channels instead of RoboClaw A.
    """

    def ensure_auto_mode(self, force: bool = False) -> None:
        super().ensure_auto_mode(force=force)

        if self.mega is None or not self._auto_entered:
            return

        resp = self.mega.send("FRONT_ROUTE MDD20A")

        if not resp.startswith("OK FRONT_ROUTE MDD20A"):
            raise RuntimeError(
                f"Failed to configure alternate front drive route: {resp}"
            )

        print(f"[MEGA ALT] {resp}")