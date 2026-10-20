"""Download this workspace's locked wheels from mirrors and verify upstream hashes."""

from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import sys
import tomllib
import urllib.parse
import urllib.request

from pip._vendor.packaging.requirements import Requirement
from pip._vendor.packaging.tags import sys_tags
from pip._vendor.packaging.utils import canonicalize_name, parse_wheel_filename


WORKSPACE = Path(__file__).resolve().parents[1]
CACHE = WORKSPACE / ".cache/wheels"


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(2 * 1024 * 1024):
            value.update(chunk)
    return value.hexdigest()


def download(item):
    filename = item["filename"]
    target = CACHE / filename
    expected = item["sha256"]
    if target.is_file() and digest(target) == expected:
        return {**item, "status": "verified-cache"}

    original = urllib.parse.urlparse(item["upstream_url"])
    if original.hostname == "files.pythonhosted.org":
        urls = ["https://pypi.tuna.tsinghua.edu.cn" + original.path]
    elif original.hostname in {"download.pytorch.org", "download-r2.pytorch.org"}:
        urls = ["https://mirror.sjtu.edu.cn/pytorch-wheels/" + original.path.removeprefix("/whl/")]
    else:
        return {**item, "status": "skipped"}

    part = target.with_suffix(target.suffix + ".part")
    for url in urls:
        for attempt in range(2):
            try:
                opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                with opener.open(url, timeout=90) as response, part.open("wb") as stream:
                    if "text/html" in response.headers.get("Content-Type", ""):
                        raise ValueError("Mirror returned HTML instead of a wheel")
                    value = hashlib.sha256()
                    while chunk := response.read(2 * 1024 * 1024):
                        stream.write(chunk)
                        value.update(chunk)
                if value.hexdigest() != expected:
                    raise ValueError("Upstream SHA-256 mismatch")
                part.replace(target)
                print(f"VERIFIED {filename}", flush=True)
                return {**item, "status": "verified-download", "mirror": urllib.parse.urlparse(url).hostname}
            except Exception as exc:
                part.unlink(missing_ok=True)
                print(f"RETRY {filename}: {type(exc).__name__}", flush=True)
    return {**item, "status": "unavailable"}


def main():
    if Path(sys.prefix) != Path("/home/pangk/miniforge3/envs/lerobot-so101"):
        raise SystemExit("Run this script in lerobot-so101")
    CACHE.mkdir(parents=True, exist_ok=True)
    requirements = []
    for line in (WORKSPACE / "logs/requirements-locked.txt").read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        req = Requirement(line)
        if req.marker is None or req.marker.evaluate():
            requirements.append(req)

    ranks = {tag: i for i, tag in enumerate(sys_tags())}
    lock = tomllib.loads((WORKSPACE / "lerobot/uv.lock").read_text())
    items = []
    for req in requirements:
        for package in lock["package"]:
            if canonicalize_name(package["name"]) != canonicalize_name(req.name):
                continue
            if not req.specifier.contains(package["version"]):
                continue
            candidates = []
            for wheel in package.get("wheels", []):
                filename = urllib.parse.unquote(urllib.parse.urlparse(wheel["url"]).path.rsplit("/", 1)[-1])
                if Path(filename).name != filename:
                    continue
                try:
                    _, _, _, tags = parse_wheel_filename(filename)
                except ValueError:
                    continue
                compatible = tags.intersection(ranks)
                if compatible:
                    candidates.append((min(ranks[tag] for tag in compatible), filename, wheel))
            if not candidates:
                continue
            _, filename, wheel = min(candidates, key=lambda row: row[0])
            items.append({
                "name": package["name"], "version": package["version"],
                "filename": filename, "sha256": wheel["hash"].split(":", 1)[-1],
                "upstream_url": wheel["url"],
            })

    print(f"PREFETCH {len(items)} locked, compatible wheels", flush=True)
    results = []
    with ThreadPoolExecutor(max_workers=8) as pool:
        for future in as_completed(pool.submit(download, item) for item in items):
            results.append(future.result())
    (WORKSPACE / "logs/mirror-wheels.json").write_text(json.dumps(results, indent=2) + "\n")
    verified = [item for item in results if item["status"].startswith("verified-")]
    specs = sorted({f"{item['name']}=={item['version']}" for item in verified})
    (WORKSPACE / "logs/cached-requirements.txt").write_text("\n".join(specs) + "\n")
    print(f"VERIFIED_TOTAL {len(verified)}/{len(items)} wheels", flush=True)


if __name__ == "__main__":
    main()
