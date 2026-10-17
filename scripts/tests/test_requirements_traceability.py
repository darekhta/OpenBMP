"""Regression cases for malformed traceability registers."""

from __future__ import annotations

import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "check_requirements_traceability.py"
SPEC = importlib.util.spec_from_file_location("check_requirements_traceability", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


class MalformedRegisterTests(unittest.TestCase):
    def check_register(self, data):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "requirements.toml").write_text("", encoding="utf-8")
            (root / "evidence.txt").write_text("evidence", encoding="utf-8")
            errors = io.StringIO()
            with (
                patch.object(checker, "__file__", str(root / "scripts" / "check.py")),
                patch.object(checker.tomllib, "load", return_value=data),
                contextlib.redirect_stderr(errors),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                result = checker.main()
            return result, errors.getvalue()

    @staticmethod
    def valid_register():
        return {
            "schema_version": 1,
            "requirements": [{
                "id": "REQ-TEST-001",
                "title": "Example",
                "statement": "Example requirement",
                "verification": ["V-TEST-001"],
            }],
            "verification": [{
                "id": "V-TEST-001",
                "kind": "test",
                "command": "example",
                "requirements": ["REQ-TEST-001"],
                "paths": ["evidence.txt"],
                "contains": ["evidence"],
            }],
        }

    def test_valid_register_remains_valid(self):
        self.assertEqual(self.check_register(self.valid_register()), (0, ""))

    def test_non_table_entries_report_errors(self):
        for field, label in (("requirements", "requirement"), ("verification", "verification")):
            with self.subTest(field=field):
                data = self.valid_register()
                data[field] = [42]
                result, errors = self.check_register(data)
                self.assertEqual(result, 1)
                self.assertIn(f"{label} #0 must be a TOML table", errors)

    def test_non_string_kind_reports_error(self):
        for kind in ([], {}, 42):
            with self.subTest(kind=kind):
                data = self.valid_register()
                data["verification"][0]["kind"] = kind
                result, errors = self.check_register(data)
                self.assertEqual(result, 1)
                self.assertIn("kind must be one of", errors)

    def test_non_list_backlinks_report_errors(self):
        for entries, field in (("requirements", "verification"), ("verification", "requirements")):
            for value in (42, "REQ-TEST-001 V-TEST-001", {}):
                with self.subTest(entries=entries, value=value):
                    data = self.valid_register()
                    data[entries][0][field] = value
                    result, errors = self.check_register(data)
                    self.assertEqual(result, 1)
                    self.assertIn(f"field {field} must be a list of strings", errors)
                    self.assertIn("does not link back", errors)


if __name__ == "__main__":
    unittest.main()
