# hw_io/clients/usb_media_client.py

from pathlib import Path


class UsbMediaClient:
    def __init__(self, search_roots=None):
        self.search_roots = search_roots or [
            Path("/media"),
            Path("/mnt"),
        ]

    def read_int(self, filename: str) -> int:
        path = self._find_file(filename)
        text = path.read_text(encoding="utf-8").strip()
        return int(text)

    def _find_file(self, filename: str) -> Path:
        for root in self.search_roots:
            if not root.exists():
                continue

            for path in root.rglob(filename):
                if path.is_file():
                    return path

        raise FileNotFoundError(
            f"Could not find {filename} on mounted USB media"
        )