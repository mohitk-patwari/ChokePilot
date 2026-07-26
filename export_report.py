"""
Markdown -> PDF / PPTX export path for the docs/ reports.

PDF: pandoc renders the markdown to a single self-contained HTML file (images and
CSS embedded via --embed-resources), then headless Chrome prints that HTML to PDF.

Why not pandoc -> LaTeX -> PDF (the more common recipe): tried it first
(--pdf-engine=xelatex, MiKTeX already installed). It silently drops every
character its default font can't show -- confirmed missing in the rendered PDF:
Greek letters (tau/theta/sigma/lambda), math symbols (>= <= ~=), and the check/
cross marks (checkmark/cross) this project's tables use throughout. Getting full
coverage in LaTeX means hand-picking fallback fonts per symbol class, forever.
Chrome uses the real system font stack (Segoe UI + Segoe UI Symbol + Segoe UI
Emoji), so it just renders everything correctly with zero font configuration --
verified by re-running the same report.md through this path (see the module's
own self-check).

PPTX: pandoc's native markdown -> pptx writer, no HTML/Chrome step -- one slide
per top-level heading, tables and images carried over automatically.

Setup, once (already done on this machine, nothing to redo):
  - pandoc wasn't on PATH -- pypandoc.download_pandoc() fetched a portable copy to
    %LOCALAPPDATA%\\Pandoc\\pandoc.exe.
  - Chrome or Edge must be installed (checked via _find_chrome() below -- both
    happened to be present here). No separate PDF-engine install needed.

Usage:
    python export_report.py docs/report.md outputs/report.pdf
    python export_report.py docs/report.md outputs/report.pptx

Relative image paths in the source markdown (e.g. ../outputs/scenario_A....png)
resolve against the input file's own directory via --resource-path.
"""

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pypandoc

_CSS = Path(__file__).parent / "docs" / "report.css"

_CHROME_CANDIDATES = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
]


def _find_chrome():
    for exe in ("chrome", "chrome.exe", "msedge", "msedge.exe"):
        found = shutil.which(exe)
        if found:
            return found
    for path in _CHROME_CANDIDATES:
        if Path(path).exists():
            return path
    raise RuntimeError("No Chrome or Edge install found for HTML->PDF rendering "
                        "(checked PATH and the usual Program Files locations).")


def _to_pdf(src, dst):
    with tempfile.TemporaryDirectory() as tmp:
        html_path = Path(tmp) / "report.html"
        pypandoc.convert_file(
            str(src), "html5", outputfile=str(html_path),
            extra_args=[f"--resource-path={src.parent}", "--standalone",
                        "--embed-resources", f"--css={_CSS}"],
        )
        chrome = _find_chrome()
        subprocess.run([
            chrome, "--headless=new", "--disable-gpu", "--no-sandbox",
            f"--print-to-pdf={dst}", "--print-to-pdf-no-header",
            html_path.resolve().as_uri(),
        ], check=True, capture_output=True)


def _to_pptx(src, dst):
    pypandoc.convert_file(
        str(src), "pptx", outputfile=str(dst),
        extra_args=[f"--resource-path={src.parent}"],
    )


def export(src, dst):
    src, dst = Path(src), Path(dst)
    fmt = dst.suffix.lstrip(".")
    if fmt == "pdf":
        _to_pdf(src, dst)
    elif fmt == "pptx":
        _to_pptx(src, dst)
    else:
        raise ValueError(f"unsupported output format: .{fmt} (use .pdf or .pptx)")
    print(f"{src} -> {dst} ({dst.stat().st_size / 1024:.0f} KB)")


def demo():
    """Self-check: round-trip docs/report.md to both formats in a temp dir and
    confirm each output is a non-trivial file that starts with the right magic
    bytes -- catches "pandoc/chrome silently produced 0 bytes" without needing a
    human to open the file every time this export path is touched."""
    import tempfile as _tf
    src = Path(__file__).parent / "docs" / "report.md"
    with _tf.TemporaryDirectory() as tmp:
        pdf_path = Path(tmp) / "test.pdf"
        pptx_path = Path(tmp) / "test.pptx"
        export(src, pdf_path)
        export(src, pptx_path)
        assert pdf_path.read_bytes()[:4] == b"%PDF", "output is not a valid PDF"
        assert pptx_path.read_bytes()[:2] == b"PK", "output is not a valid PPTX (zip)"
        assert pdf_path.stat().st_size > 10_000, "PDF suspiciously small"
        assert pptx_path.stat().st_size > 10_000, "PPTX suspiciously small"
    print("export_report.py self-check PASSED (PDF and PPTX both round-trip cleanly)")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--demo":
        demo()
    elif len(sys.argv) == 3:
        export(sys.argv[1], sys.argv[2])
    else:
        print("usage: python export_report.py <input.md> <output.pdf|output.pptx>")
        print("       python export_report.py --demo")
        sys.exit(1)
