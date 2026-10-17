#!/usr/bin/env python3
"""Validate run-data CZML/glTF export goldens against upstream JSON schemas."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


GLTF_REPO = "KhronosGroup/glTF"
GLTF_COMMIT = "5382af51376ef5bae767b42e9b2202b3cab9a1b4"
GLTF_SCHEMA_PREFIX = "specification/2.0/schema"

CZML_REPO = "AnalyticalGraphicsInc/czml-writer"
CZML_COMMIT = "7d37f31846c05c5931b205f6434f9bd275587c0e"
CZML_SCHEMA_PREFIX = "Schema"

AJV_VERSION = "8.17.1"

VALIDATOR_JS = r"""
const fs = require("fs");
const Ajv = require("ajv");
const Ajv2020Module = require("ajv/dist/2020");
const Ajv2020 = Ajv2020Module.default || Ajv2020Module;

function readJson(path) {
  return JSON.parse(fs.readFileSync(path, "utf8"));
}

const [spec, schemaPath, dataPath, ...refPaths] = process.argv.slice(2);
const ajvOptions = { strict: false, allErrors: true, validateFormats: false };
const ajv = spec === "2020" ? new Ajv2020(ajvOptions) : new Ajv(ajvOptions);

for (const refPath of refPaths) {
  if (refPath !== schemaPath) {
    const schema = readJson(refPath);
    if (schema && !Array.isArray(schema) && typeof schema === "object") {
      ajv.addSchema(schema, schema.$id || refPath);
    }
  }
}

const validate = ajv.compile(readJson(schemaPath));
const valid = validate(readJson(dataPath));
if (!valid) {
  console.error(JSON.stringify(validate.errors, null, 2));
  process.exit(1);
}
"""


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    npm = shutil.which("npm")
    node = shutil.which("node")
    if npm is None or node is None:
        print(
            "export3d schemas: node and npm are required for AJV validation",
            file=sys.stderr,
        )
        return 1

    czml_golden = root / "crates/openbmp-rundata/tests/fixtures/export3d.czml.json"
    gltf_golden = root / "crates/openbmp-rundata/tests/fixtures/export3d.gltf.json"
    if not czml_golden.is_file() or not gltf_golden.is_file():
        print("export3d schemas: expected CZML/glTF goldens are missing", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="openbmp-export3d-schema-") as tmp:
        tmp_path = Path(tmp)
        node_dir = tmp_path / "node"
        schema_dir = tmp_path / "schema"
        node_dir.mkdir()
        schema_dir.mkdir()

        install_ajv(npm=npm, cwd=node_dir)
        validator = node_dir / "validate.js"
        validator.write_text(VALIDATOR_JS, encoding="utf-8")

        gltf_schema_dir = schema_dir / "gltf"
        czml_schema_dir = schema_dir / "czml"
        gltf_schemas = download_schema_tree(
            repo=GLTF_REPO,
            commit=GLTF_COMMIT,
            prefix=GLTF_SCHEMA_PREFIX,
            destination=gltf_schema_dir,
        )
        czml_schemas = download_schema_tree(
            repo=CZML_REPO,
            commit=CZML_COMMIT,
            prefix=CZML_SCHEMA_PREFIX,
            destination=czml_schema_dir,
        )
        czml_wrapper = schema_dir / "czml-document.schema.json"
        czml_wrapper.write_text(
            json.dumps(
                {
                    "$schema": "http://json-schema.org/draft-07/schema#",
                    "type": "array",
                    "items": {
                        "$ref": "https://analyticalgraphicsinc.github.io/czml-writer/Schema/Packet.json"
                    },
                },
                sort_keys=True,
            ),
            encoding="utf-8",
        )

        validate_json(
            node=node,
            cwd=node_dir,
            validator=validator,
            spec="2020",
            schema=gltf_schema_dir / "glTF.schema.json",
            data=gltf_golden,
            refs=gltf_schemas,
        )
        validate_json(
            node=node,
            cwd=node_dir,
            validator=validator,
            spec="draft7",
            schema=czml_wrapper,
            data=czml_golden,
            refs=czml_schemas,
        )

    print("export3d schemas: official glTF 2.0 and CZML JSON schemas validated")
    return 0


def install_ajv(*, npm: str, cwd: Path) -> None:
    run(
        [
            npm,
            "install",
            "--silent",
            "--no-audit",
            "--no-fund",
            "--no-save",
            f"ajv@{AJV_VERSION}",
        ],
        cwd=cwd,
    )


def download_schema_tree(
    *,
    repo: str,
    commit: str,
    prefix: str,
    destination: Path,
) -> list[Path]:
    destination.mkdir(parents=True, exist_ok=True)
    tree_url = f"https://api.github.com/repos/{repo}/git/trees/{commit}?recursive=1"
    tree = json.loads(download(tree_url).decode("utf-8"))
    paths: list[Path] = []
    for entry in tree.get("tree", []):
        path = entry.get("path", "")
        if (
            entry.get("type") != "blob"
            or not path.startswith(f"{prefix}/")
            or not path.endswith(".json")
        ):
            continue
        relative = Path(path).relative_to(prefix)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        raw_path = urllib.parse.quote(path, safe="/")
        raw_url = f"https://raw.githubusercontent.com/{repo}/{commit}/{raw_path}"
        target.write_bytes(download(raw_url))
        paths.append(target)
    if not paths:
        raise SystemExit(f"export3d schemas: no JSON schemas found in {repo}/{prefix}")
    return sorted(paths)


def download(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "OpenBMP export3d schema gate"})
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return response.read()
    except (TimeoutError, urllib.error.URLError) as exc:
        raise SystemExit(f"export3d schemas: could not download {url}: {exc}") from exc


def validate_json(
    *,
    node: str,
    cwd: Path,
    validator: Path,
    spec: str,
    schema: Path,
    data: Path,
    refs: list[Path],
) -> None:
    run(
        [
            node,
            str(validator),
            spec,
            str(schema),
            str(data),
            *[str(path) for path in refs if path != schema],
        ],
        cwd=cwd,
    )


def run(command: list[str], *, cwd: Path) -> None:
    result = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise SystemExit(result.returncode)


if __name__ == "__main__":
    raise SystemExit(main())
