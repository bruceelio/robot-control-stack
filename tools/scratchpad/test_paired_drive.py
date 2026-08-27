from config import CONFIG
from hw_io.resolve import resolve_io


def main() -> None:
    io = resolve_io(
        robot=None,
    )

    try:
        print("Starting paired front drive test")

        io.drive["front"].set_power(
            left=0.20,
            right=0.20,
        )

        io.sleep(1.0)

        io.drive["front"].set_power(
            left=0.0,
            right=0.0,
        )

        print("Paired front drive test complete")

    finally:
        try:
            io.drive["front"].set_power(
                left=0.0,
                right=0.0,
            )
        finally:
            close = getattr(io, "close", None)
            if callable(close):
                close()


if __name__ == "__main__":
    main()