#!/usr/bin/env python3
"""Train a fully connected neural network for glove position classification.

Dataset format (space-separated):
    col1 col2 position

Where:
- col1, col2 are the two analog features (A1, A0 in your current setup)
- position is the class label (0..8)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import joblib
import numpy as np
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix, f1_score
from sklearn.model_selection import GridSearchCV, StratifiedKFold, train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler


def load_dataset(dataset_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load 3-column dataset and return X (first 2 cols), y (third col)."""
    data = np.loadtxt(dataset_path, dtype=float)

    # Handle edge case where file has only one row.
    if data.ndim == 1:
        if data.shape[0] != 3:
            raise ValueError("Dataset must have exactly 3 columns per row.")
        data = data.reshape(1, 3)

    if data.shape[1] != 3:
        raise ValueError("Dataset must have exactly 3 columns: feature1 feature2 label")

    x = data[:, :2]
    y = data[:, 2].astype(int)
    return x, y


def build_model(random_state: int) -> Pipeline:
    """Create a fully connected neural network pipeline."""
    return Pipeline(
        steps=[
            ("scaler", StandardScaler()),
            (
                "mlp",
                MLPClassifier(
                    hidden_layer_sizes=(64, 64),
                    activation="relu",
                    solver="adam",
                    alpha=1e-4,
                    learning_rate_init=1e-3,
                    max_iter=3000,
                    early_stopping=False,
                    random_state=random_state,
                ),
            ),
        ]
    )


def main() -> None:
    default_dataset = Path(__file__).with_name("dataset_glove_2in_9pos_new.txt")
    default_model = Path(__file__).with_name("glove_fcnn_model.joblib")

    parser = argparse.ArgumentParser(description="Train FCNN for glove position prediction")
    parser.add_argument("--dataset", type=Path, default=default_dataset, help="Path to dataset txt")
    parser.add_argument("--model-out", type=Path, default=default_model, help="Output model path")
    parser.add_argument("--test-size", type=float, default=0.2, help="Test split ratio (0-1)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--cv-folds", type=int, default=5, help="Stratified CV folds")
    args = parser.parse_args()

    x, y = load_dataset(args.dataset)

    x_train, x_test, y_train, y_test = train_test_split(
        x,
        y,
        test_size=args.test_size,
        random_state=args.seed,
        stratify=y,
    )

    base_model = build_model(random_state=args.seed)

    # Tune FCNN hyperparameters for balanced class performance.
    param_grid = {
        "mlp__hidden_layer_sizes": [(40, 20),(64, 32), (128, 64), (128, 128)],
        "mlp__alpha": [1e-5, 1e-4, 1e-3],
        "mlp__learning_rate_init": [3e-4, 1e-3, 3e-3],
    }
    cv = StratifiedKFold(n_splits=args.cv_folds, shuffle=True, random_state=args.seed)
    search = GridSearchCV(
        estimator=base_model,
        param_grid=param_grid,
        scoring="f1_macro",
        cv=cv,
        n_jobs=-1,
        refit=True,
    )
    search.fit(x_train, y_train)
    model = search.best_estimator_

    y_pred = model.predict(x_test)
    acc = accuracy_score(y_test, y_pred)
    macro_f1 = f1_score(y_test, y_pred, average="macro")

    print(f"Dataset: {args.dataset}")
    print(f"Samples: {len(y)} | Train: {len(y_train)} | Test: {len(y_test)}")
    print(f"Best CV macro-F1: {search.best_score_:.4f}")
    print(f"Best params: {search.best_params_}")
    print(f"Test accuracy: {acc:.4f}")
    print(f"Test macro-F1: {macro_f1:.4f}")
    print("\nClassification report:")
    print(classification_report(y_test, y_pred, digits=4, zero_division=0))
    print("Confusion matrix (rows=true, cols=pred):")
    print(confusion_matrix(y_test, y_pred))

    args.model_out.parent.mkdir(parents=True, exist_ok=True)
    joblib.dump(
        {
            "model": model,
            "feature_order": ["A1", "A0"],
            "label_name": "position",
            "classes": np.unique(y),
            "best_params": search.best_params_,
            "best_cv_macro_f1": float(search.best_score_),
        },
        args.model_out,
    )
    print(f"\nSaved model to: {args.model_out}")


if __name__ == "__main__":
    main()
