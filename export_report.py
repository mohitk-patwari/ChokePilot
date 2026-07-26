"""
Markdown -> PDF / PPTX export path for the docs/ reports.

PDF: pandoc renders the markdown to a single self-contained HTML file (images and
CSS embedded via --embed-resources), then headless Chrome prints that HTML to PDF.
Falls back to pandoc -> LaTeX (xelatex) -> PDF if no Chrome/Edge install is found
or the Chrome path fails for any reason (see `_to_pdf`) -- tried LaTeX first
(--pdf-engine=xelatex, MiKTeX already installed) and it silently drops every
character its default font can't show: Greek letters (tau/theta/sigma/lambda),
math symbols (>= <= ~=), and the check/cross marks this project's tables use
throughout, confirmed missing by visual inspection of a rendered page. Getting
full coverage in LaTeX means hand-picking fallback fonts per symbol class,
forever. Chrome uses the real system font stack (Segoe UI + Segoe UI Symbol +
Segoe UI Emoji) and just renders everything correctly with zero font
configuration, so it's the preferred path -- xelatex is kept only as a
still-produces-*a*-PDF fallback for a machine without a browser installed, not
the primary path.

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


def _to_pdf_chrome(src, dst):
    """Primary path: pandoc -> self-contained HTML -> headless Chrome/Edge print.
    Full Unicode/emoji coverage via the real system font stack (see module
    docstring) -- preferred whenever Chrome or Edge is present."""
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


def _to_pdf_xelatex(src, dst):
    """Fallback path: pandoc -> LaTeX -> PDF via xelatex (MiKTeX). Used only when
    no Chrome/Edge install can be found. Known gap, accepted for a fallback: the
    default LaTeX font is missing glyphs for Greek letters, some math symbols, and
    check/cross marks -- those render as blank boxes rather than crashing the
    build, which is the right tradeoff for "still produce *a* PDF" under time
    pressure, not for the final shipped version."""
    pypandoc.convert_file(
        str(src), "pdf", outputfile=str(dst),
        extra_args=[f"--resource-path={src.parent}", "--pdf-engine=xelatex",
                     "-V", "geometry:margin=1in", "--standalone"],
    )


def _to_pdf(src, dst):
    """Try the Chrome/Edge path first; fall back to xelatex if no browser is
    found or the browser path fails for any reason -- packaging a submission
    should never hard-fail just because this machine lacks a browser install."""
    try:
        _find_chrome()
    except RuntimeError as e:
        print(f"  [PDF] no Chrome/Edge found ({e}) -- falling back to xelatex")
        _to_pdf_xelatex(src, dst)
        return "xelatex"
    try:
        _to_pdf_chrome(src, dst)
        return "chrome"
    except Exception as e:
        print(f"  [PDF] Chrome/Edge render failed ({e}) -- falling back to xelatex")
        _to_pdf_xelatex(src, dst)
        return "xelatex"


def _to_pptx(src, dst):
    pypandoc.convert_file(
        str(src), "pptx", outputfile=str(dst),
        extra_args=[f"--resource-path={src.parent}"],
    )


def export(src, dst):
    src, dst = Path(src), Path(dst)
    fmt = dst.suffix.lstrip(".")
    engine = None
    if fmt == "pdf":
        engine = _to_pdf(src, dst)
    elif fmt == "pptx":
        _to_pptx(src, dst)
    else:
        raise ValueError(f"unsupported output format: .{fmt} (use .pdf or .pptx)")
    tag = f" [{engine}]" if engine else ""
    print(f"{src} -> {dst}{tag} ({dst.stat().st_size / 1024:.0f} KB)")
    return engine


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
