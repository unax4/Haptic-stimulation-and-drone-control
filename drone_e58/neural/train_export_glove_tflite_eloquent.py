#!/usr/bin/env python3
"""Train and export glove FCNN as quantized TensorFlow Lite for Arduino Eloquent.

This script:
1) loads a 2-feature dataset (A1, A0, label)
2) trains an FCNN with the best params found in sklearn search
3) converts and quantizes to int8 TFLite
4) writes Arduino-ready model bytes header

Best params used by default:
    alpha = 1e-5
    hidden_layer_sizes = (40, 20)
    learning_rate_init = 1e-3
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import tensorflow as tf
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix, f1_score
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler


def load_dataset(dataset_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load 3-column dataset and return X (first 2 cols), y (third col)."""
    data = np.loadtxt(dataset_path, dtype=float)

    if data.ndim == 1:
        if data.shape[0] != 3:
            raise ValueError("Dataset must have exactly 3 columns per row.")
        data = data.reshape(1, 3)

    if data.shape[1] != 3:
        raise ValueError("Dataset must have exactly 3 columns: feature1 feature2 label")

    x = data[:, :2].astype(np.float32)
    y = data[:, 2].astype(np.int32)
    return x, y


def build_model(alpha: float, learning_rate: float, num_classes: int) -> tf.keras.Model:
    """Build FCNN architecture 2 -> 40 -> 20 -> num_classes."""
    l2 = tf.keras.regularizers.L2(alpha)
    model = tf.keras.Sequential(
        [
            tf.keras.layers.Input(shape=(2,), name="input"),
            tf.keras.layers.Dense(40, activation="relu", kernel_regularizer=l2, name="dense_40"),
            tf.keras.layers.Dense(20, activation="relu", kernel_regularizer=l2, name="dense_20"),
            tf.keras.layers.Dense(num_classes, activation="softmax", name="output"),
        ],
        name="glove_fcnn_40_20",
    )

    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=learning_rate),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )
    return model


def representative_dataset_gen(x_scaled: np.ndarray):
    """Yield representative samples for post-training int8 quantization."""
    for i in range(min(len(x_scaled), 300)):
        sample = x_scaled[i : i + 1].astype(np.float32)
        yield [sample]


def tflite_bytes_to_header(model_bytes: bytes, array_name: str = "g_glove_model") -> str:
    """Convert TFLite bytes into a C header string."""
    hex_rows = []
    row = []
    for i, b in enumerate(model_bytes):
        row.append(f"0x{b:02x}")
        if (i + 1) % 12 == 0:
            hex_rows.append(", ".join(row))
            row = []
    if row:
        hex_rows.append(", ".join(row))

    body = ",\n  ".join(hex_rows)
    return (
        "#pragma once\n"
        "#include <Arduino.h>\n\n"
        f"alignas(16) const unsigned char {array_name}[] = {{\n"
        f"  {body}\n"
        "};\n"
        f"const unsigned int {array_name}_len = {len(model_bytes)};\n"
    )


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    default_dataset = script_dir / "dataset_glove_2in_9pos_new.txt"
    default_tflite = script_dir / "glove_fcnn_40_20_int8.tflite"
    default_header = script_dir / "glove_fcnn_40_20_model_data.h"

    parser = argparse.ArgumentParser(description="Train FCNN and export quantized TFLite for Arduino")
    parser.add_argument("--dataset", type=Path, default=default_dataset, help="Path to dataset txt")
    parser.add_argument("--tflite-out", type=Path, default=default_tflite, help="Output .tflite path")
    parser.add_argument("--header-out", type=Path, default=default_header, help="Output C header path")
    parser.add_argument("--test-size", type=float, default=0.2, help="Test split ratio (0-1)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--epochs", type=int, default=120, help="Training epochs")
    parser.add_argument("--batch-size", type=int, default=16, help="Batch size")
    parser.add_argument("--val-split", type=float, default=0.2, help="Validation split from train set")
    args = parser.parse_args()

    # Best params from your sklearn search.
    alpha = 1e-5
    hidden_layer_sizes = (40, 20)
    learning_rate_init = 1e-3

    print("Selected NN architecture: FCNN")
    print(f"Hidden layers: {hidden_layer_sizes}")
    print(f"Best params: {{'mlp__alpha': {alpha}, 'mlp__hidden_layer_sizes': {hidden_layer_sizes}, 'mlp__learning_rate_init': {learning_rate_init}}}")

    x, y = load_dataset(args.dataset)
    classes = np.unique(y)
    num_classes = int(classes.max()) + 1

    x_train, x_test, y_train, y_test = train_test_split(
        x,
        y,
        test_size=args.test_size,
        random_state=args.seed,
        stratify=y,
    )

    scaler = StandardScaler()
    x_train_scaled = scaler.fit_transform(x_train).astype(np.float32)
    x_test_scaled = scaler.transform(x_test).astype(np.float32)

    tf.keras.utils.set_random_seed(args.seed)
    model = build_model(alpha=alpha, learning_rate=learning_rate_init, num_classes=num_classes)
    model.fit(
        x_train_scaled,
        y_train,
        validation_split=args.val_split,
        epochs=args.epochs,
        batch_size=args.batch_size,
        verbose=0,
    )

    probs = model.predict(x_test_scaled, verbose=0)
    y_pred = np.argmax(probs, axis=1)

    acc = accuracy_score(y_test, y_pred)
    macro_f1 = f1_score(y_test, y_pred, average="macro")
    print(f"Dataset: {args.dataset}")
    print(f"Samples: {len(y)} | Train: {len(y_train)} | Test: {len(y_test)}")
    print(f"Test accuracy: {acc:.4f}")
    print(f"Test macro-F1: {macro_f1:.4f}")
    print("\nClassification report:")
    print(classification_report(y_test, y_pred, digits=4, zero_division=0))
    print("Confusion matrix (rows=true, cols=pred):")
    print(confusion_matrix(y_test, y_pred))

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = lambda: representative_dataset_gen(x_train_scaled)
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    tflite_model = converter.convert()

    args.tflite_out.parent.mkdir(parents=True, exist_ok=True)
    args.header_out.parent.mkdir(parents=True, exist_ok=True)

    args.tflite_out.write_bytes(tflite_model)
    args.header_out.write_text(tflite_bytes_to_header(tflite_model), encoding="utf-8")

    interpreter = tf.lite.Interpreter(model_content=tflite_model)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()[0]
    output_details = interpreter.get_output_details()[0]
    in_scale, in_zero = input_details["quantization"]
    out_scale, out_zero = output_details["quantization"]

    print("\nTFLite int8 quantization:")
    print(f"Input scale={in_scale}, zero_point={in_zero}")
    print(f"Output scale={out_scale}, zero_point={out_zero}")
    print(f"Saved TFLite model: {args.tflite_out}")
    print(f"Saved Arduino header: {args.header_out}")
    print("\nScaler values for Arduino preprocessing:")
    print(f"mean=[{scaler.mean_[0]:.8f}, {scaler.mean_[1]:.8f}]")
    print(f"scale=[{scaler.scale_[0]:.8f}, {scaler.scale_[1]:.8f}]")


if __name__ == "__main__":
    main()
