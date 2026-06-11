#!/usr/bin/env python3
"""check_fmi_export_schema.py: validate generated FMI export XML with XSD gates."""

from __future__ import annotations

import hashlib
import shutil
import subprocess
import sys
import tempfile
import textwrap
import urllib.error
import urllib.request
from pathlib import Path


FMI_SCHEMA_BASE_URL = (
    "https://raw.githubusercontent.com/modelica/fmi-standard/v3.0/schema"
)
OFFICIAL_FMI3_SCHEMA_FILES = {
    "fmi3Annotation.xsd": "7c871dfcee048cfa0ae5a70d6774d3e9afbe180c70b8713c1bc6cba0838c5d4f",
    "fmi3AttributeGroups.xsd": "64136fa765685b041ff9e5fbc4a4f0f62b76e064b2b03fea3ab650c0f64f6586",
    "fmi3BuildDescription.xsd": "39b06e6bd56210328aa68e88ec8bc5bea4f352608bc55b18dd2b0866ac5692b4",
    "fmi3InterfaceType.xsd": "c27d4c76f3ce6495973fe860f9d88c7456353543e49625b39402140e7c1654d5",
    "fmi3ModelDescription.xsd": "51930265f2677cf059134357354b681b18d861bf0a03da1f383b8b4f60ac1394",
    "fmi3Terminal.xsd": "67985822c7ad327c66295b101d5bac2811a967703b01dde5d8d3c80e1db50827",
    "fmi3TerminalsAndIcons.xsd": "5fded084be5188da4f8b124d3a5d970aee6ff3622ea724c30c683f319c7b1ce4",
    "fmi3Type.xsd": "49653058c9291f8ca604b3ce24a6e11e39da29885ba7297c3ed7f681415fb9b7",
    "fmi3Unit.xsd": "413e4dcd7675b35812711b876477600a23a539fd3a960067037aae1030863f30",
    "fmi3Variable.xsd": "0c94521154bc97c5c8bd8a14500eb1b8241fe42e3c8f9ab7d2803bc80cb315f0",
    "fmi3VariableDependency.xsd": "b17bd03a0c68d7aa0abe5485865ff5c54f81e6ec89f78d7dc7619acdce939c81",
}

RESTRICTED_FMI3_XSD = textwrap.dedent(
    """\
    <?xml version="1.0" encoding="UTF-8"?>
    <xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema"
               elementFormDefault="unqualified"
               attributeFormDefault="unqualified">
      <xs:simpleType name="fmiVersionType">
        <xs:restriction base="xs:string">
          <xs:enumeration value="3.0"/>
        </xs:restriction>
      </xs:simpleType>

      <xs:simpleType name="causalityType">
        <xs:restriction base="xs:string">
          <xs:enumeration value="parameter"/>
          <xs:enumeration value="calculatedParameter"/>
          <xs:enumeration value="input"/>
          <xs:enumeration value="output"/>
          <xs:enumeration value="local"/>
          <xs:enumeration value="independent"/>
        </xs:restriction>
      </xs:simpleType>

      <xs:complexType name="scalarVariableType">
        <xs:attribute name="name" type="xs:string" use="required"/>
        <xs:attribute name="valueReference" type="xs:unsignedInt" use="required"/>
        <xs:attribute name="causality" type="causalityType" use="optional"/>
        <xs:attribute name="initial" type="initialType" use="optional"/>
        <xs:attribute name="start" type="xs:string" use="optional"/>
      </xs:complexType>

      <xs:simpleType name="initialType">
        <xs:restriction base="xs:string">
          <xs:enumeration value="exact"/>
          <xs:enumeration value="approx"/>
          <xs:enumeration value="calculated"/>
        </xs:restriction>
      </xs:simpleType>

      <xs:complexType name="coSimulationType">
        <xs:attribute name="modelIdentifier" type="xs:string" use="required"/>
        <xs:attribute name="canHandleVariableCommunicationStepSize" type="xs:boolean" use="optional"/>
        <xs:attribute name="canGetAndSetFMUState" type="xs:boolean" use="optional"/>
      </xs:complexType>

      <xs:complexType name="modelVariablesType">
        <xs:choice minOccurs="1" maxOccurs="unbounded">
          <xs:element name="Float64" type="scalarVariableType"/>
          <xs:element name="UInt64" type="scalarVariableType"/>
        </xs:choice>
      </xs:complexType>

      <xs:complexType name="modelStructureType">
        <xs:sequence>
          <xs:element name="Output" minOccurs="0" maxOccurs="unbounded">
            <xs:complexType>
              <xs:attribute name="valueReference" type="xs:unsignedInt" use="required"/>
            </xs:complexType>
          </xs:element>
        </xs:sequence>
      </xs:complexType>

      <xs:element name="fmiModelDescription">
        <xs:complexType>
          <xs:sequence>
            <xs:element name="CoSimulation" type="coSimulationType"/>
            <xs:element name="ModelVariables" type="modelVariablesType"/>
            <xs:element name="ModelStructure" type="modelStructureType"/>
          </xs:sequence>
          <xs:attribute name="fmiVersion" type="fmiVersionType" use="required"/>
          <xs:attribute name="modelName" type="xs:string" use="required"/>
          <xs:attribute name="instantiationToken" type="xs:string" use="required"/>
          <xs:attribute name="variableNamingConvention" type="xs:string" use="optional"/>
        </xs:complexType>
      </xs:element>
    </xs:schema>
    """
)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    xmllint = shutil.which("xmllint")
    if xmllint is None:
        print(
            "fmi export schema: xmllint is required; install libxml2-utils",
            file=sys.stderr,
        )
        return 1

    xml = generate_model_description(root)
    with tempfile.TemporaryDirectory(prefix="openbmp-fmi-schema-") as tmp:
        tmp_path = Path(tmp)
        xml_path = tmp_path / "modelDescription.xml"
        xsd_path = tmp_path / "restricted-fmi3-export.xsd"
        xml_path.write_text(xml, encoding="utf-8")
        xsd_path.write_text(RESTRICTED_FMI3_XSD, encoding="utf-8")

        validate_with_xmllint(
            xmllint=xmllint,
            schema_path=xsd_path,
            xml_path=xml_path,
            cwd=root,
        )
        try:
            official_schema_path = download_official_fmi3_schemas(tmp_path)
        except OfficialSchemaError as exc:
            print(f"fmi export schema: {exc}", file=sys.stderr)
            return 1
        validate_with_xmllint(
            xmllint=xmllint,
            schema_path=official_schema_path,
            xml_path=xml_path,
            cwd=root,
        )

    print("fmi export schema: restricted and official FMI 3.0 XSD validated")
    return 0


class OfficialSchemaError(RuntimeError):
    """Official FMI schema download or integrity validation failed."""


def download_official_fmi3_schemas(destination: Path) -> Path:
    for name, expected_sha256 in OFFICIAL_FMI3_SCHEMA_FILES.items():
        url = f"{FMI_SCHEMA_BASE_URL}/{name}"
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                payload = response.read()
        except (TimeoutError, urllib.error.URLError) as exc:
            raise OfficialSchemaError(f"could not download {url}: {exc}") from exc

        actual_sha256 = hashlib.sha256(payload).hexdigest()
        if actual_sha256 != expected_sha256:
            raise OfficialSchemaError(
                f"{name} SHA-256 mismatch: expected {expected_sha256}, got {actual_sha256}"
            )
        (destination / name).write_bytes(payload)
    return destination / "fmi3ModelDescription.xsd"


def validate_with_xmllint(
    *,
    xmllint: str,
    schema_path: Path,
    xml_path: Path,
    cwd: Path,
) -> None:
    result = subprocess.run(
        [xmllint, "--noout", "--schema", str(schema_path), str(xml_path)],
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise SystemExit(result.returncode)


def generate_model_description(root: Path) -> str:
    result = subprocess.run(
        [
            "cargo",
            "run",
            "--quiet",
            "--locked",
            "-p",
            "openbmp-fmi",
            "--example",
            "export_point_mass_model_description",
        ],
        cwd=root,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise SystemExit(result.returncode)
    return result.stdout


if __name__ == "__main__":
    raise SystemExit(main())
