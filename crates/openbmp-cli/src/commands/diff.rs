//! `openbmp diff <golden.parquet> <actual.parquet>` — first divergent
//! row.
//!
//! The diff is row-by-row, column-by-column. We require both archives to
//! agree on column names and types up to the schema-version metadata. On
//! the first row that differs in any column, we emit a structured
//! description and stop.

use std::fs::File;
use std::path::{Path, PathBuf};

use arrow::array::{Array, BooleanArray, Float64Array, Int64Array, StringArray, UInt64Array};
use arrow::record_batch::RecordBatch;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

use crate::error::CliError;

/// Outcome of `openbmp diff`.
#[derive(Debug)]
pub struct DiffReport {
    /// `true` when the two archives are identical row-for-row,
    /// column-for-column.
    pub identical: bool,
    /// Row count seen on the golden side (sum across batches).
    pub golden_rows: usize,
    /// Row count seen on the actual side.
    pub actual_rows: usize,
    /// First divergence, if any.
    pub first_divergence: Option<Divergence>,
}

/// First divergent cell.
#[derive(Debug, Clone, PartialEq)]
pub struct Divergence {
    /// Row index (0-based) where divergence was first seen.
    pub row: usize,
    /// Column name.
    pub column: String,
    /// Stringified golden value (or `"<absent>"`).
    pub golden: String,
    /// Stringified actual value (or `"<absent>"`).
    pub actual: String,
}

/// Entry point.
///
/// # Errors
///
/// Returns [`CliError::Diff`] when the archives differ, plus parquet /
/// arrow read errors.
pub fn run(golden: &Path, actual: &Path) -> Result<DiffReport, CliError> {
    let golden_table = read_parquet(golden)?;
    let actual_table = read_parquet(actual)?;

    if golden_table.column_names != actual_table.column_names {
        let golden_cols = golden_table.column_names.join(",");
        let actual_cols = actual_table.column_names.join(",");
        return Err(CliError::Diff {
            summary: format!("column set differs: golden=[{golden_cols}], actual=[{actual_cols}]"),
        });
    }

    let row_count = golden_table.row_count.min(actual_table.row_count);
    for row in 0..row_count {
        for (col_index, name) in golden_table.column_names.iter().enumerate() {
            let golden_value = golden_table.cell(col_index, row);
            let actual_value = actual_table.cell(col_index, row);
            if golden_value != actual_value {
                let divergence = Divergence {
                    row,
                    column: name.clone(),
                    golden: golden_value,
                    actual: actual_value,
                };
                return Ok(DiffReport {
                    identical: false,
                    golden_rows: golden_table.row_count,
                    actual_rows: actual_table.row_count,
                    first_divergence: Some(divergence),
                });
            }
        }
    }

    if golden_table.row_count != actual_table.row_count {
        return Ok(DiffReport {
            identical: false,
            golden_rows: golden_table.row_count,
            actual_rows: actual_table.row_count,
            first_divergence: Some(Divergence {
                row: row_count,
                column: "<row-count>".to_owned(),
                golden: golden_table.row_count.to_string(),
                actual: actual_table.row_count.to_string(),
            }),
        });
    }

    Ok(DiffReport {
        identical: true,
        golden_rows: golden_table.row_count,
        actual_rows: actual_table.row_count,
        first_divergence: None,
    })
}

struct DecodedTable {
    column_names: Vec<String>,
    row_count: usize,
    columns: Vec<Vec<String>>,
}

impl DecodedTable {
    fn cell(&self, col: usize, row: usize) -> String {
        self.columns
            .get(col)
            .and_then(|column| column.get(row))
            .cloned()
            .unwrap_or_else(|| "<absent>".to_owned())
    }
}

fn read_parquet(path: &Path) -> Result<DecodedTable, CliError> {
    let file = File::open(path).map_err(|source| CliError::Io {
        path: PathBuf::from(path),
        source,
    })?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let schema = builder.schema().clone();

    let column_names = schema
        .fields()
        .iter()
        .map(|field| field.name().clone())
        .collect::<Vec<_>>();
    let column_count = column_names.len();
    let mut columns: Vec<Vec<String>> = vec![Vec::new(); column_count];

    let reader = builder.build()?;
    let mut row_count = 0;
    for batch in reader {
        let batch: RecordBatch = batch?;
        row_count += batch.num_rows();
        for (col, column) in columns.iter_mut().enumerate().take(column_count) {
            let array = batch.column(col);
            decode_column(array.as_ref(), column);
        }
    }

    Ok(DecodedTable {
        column_names,
        row_count,
        columns,
    })
}

fn decode_column(array: &dyn Array, into: &mut Vec<String>) {
    if let Some(values) = array.as_any().downcast_ref::<Float64Array>() {
        for i in 0..values.len() {
            into.push(if values.is_null(i) {
                "<null>".to_owned()
            } else {
                let bits = values.value(i).to_bits();
                format!("f64:0x{bits:016x}")
            });
        }
    } else if let Some(values) = array.as_any().downcast_ref::<UInt64Array>() {
        for i in 0..values.len() {
            into.push(if values.is_null(i) {
                "<null>".to_owned()
            } else {
                format!("u64:{}", values.value(i))
            });
        }
    } else if let Some(values) = array.as_any().downcast_ref::<Int64Array>() {
        for i in 0..values.len() {
            into.push(if values.is_null(i) {
                "<null>".to_owned()
            } else {
                format!("i64:{}", values.value(i))
            });
        }
    } else if let Some(values) = array.as_any().downcast_ref::<BooleanArray>() {
        for i in 0..values.len() {
            into.push(if values.is_null(i) {
                "<null>".to_owned()
            } else {
                format!("bool:{}", values.value(i))
            });
        }
    } else if let Some(values) = array.as_any().downcast_ref::<StringArray>() {
        for i in 0..values.len() {
            into.push(if values.is_null(i) {
                "<null>".to_owned()
            } else {
                format!("text:{}", values.value(i))
            });
        }
    } else {
        for _ in 0..array.len() {
            into.push(format!("<unsupported:{}>", array.data_type()));
        }
    }
}
