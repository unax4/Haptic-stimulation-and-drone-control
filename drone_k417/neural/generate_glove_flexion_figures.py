#!/usr/bin/env python3
"""Generate figures for the glove finger-flexion processing section.

The script reads the two-input, nine-position glove dataset and reproduces the
architecture selection method used in ``train_glove_fcnn.py``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import accuracy_score, f1_score
from sklearn.model_selection import GridSearchCV, StratifiedKFold, train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler


def load_dataset(dataset_path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(dataset_path, dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 3:
        raise ValueError("Dataset must have exactly 3 columns: feature1 feature2 label")
    return data[:, :2], data[:, 2].astype(int)


def build_model(random_state: int) -> Pipeline:
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


def save_histogram(y: np.ndarray, output_path: Path) -> None:
    labels, counts = np.unique(y, return_counts=True)

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    bars = ax.bar(labels, counts, color="#4C78A8", edgecolor="#26384f", linewidth=0.8)
    ax.axhline(counts.mean(), color="#D55E00", linestyle="--", linewidth=1.4, label="Media")
    ax.set_xlabel("Posición")
    ax.set_ylabel("Número de muestras")
    ax.set_xticks(labels)
    ax.set_ylim(0, max(counts) + 8)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)

    for bar, count in zip(bars, counts):
        ax.text(bar.get_x() + bar.get_width() / 2, count + 1, str(count), ha="center", va="bottom")

    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def save_boxplot(x: np.ndarray, y: np.ndarray, output_path: Path) -> None:
    labels = np.unique(y)
    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.4), sharey=True)
    names = ["Entrada 1 (A1)", "Entrada 2 (A0)"]

    for axis, feature_index, name in zip(axes, [0, 1], names):
        values = [x[y == label, feature_index] for label in labels]
        axis.boxplot(
            values,
            labels=[str(label) for label in labels],
            patch_artist=True,
            medianprops={"color": "#D55E00", "linewidth": 1.4},
            boxprops={"facecolor": "#A6CEE3", "edgecolor": "#26384f"},
            whiskerprops={"color": "#26384f"},
            capprops={"color": "#26384f"},
            flierprops={"marker": ".", "markersize": 3, "markerfacecolor": "#26384f"},
        )
        axis.set_title(name)
        axis.set_xlabel("Posición")
        axis.grid(axis="y", alpha=0.25)

    axes[0].set_ylabel("Lectura ADC")
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def run_architecture_search(
    x: np.ndarray,
    y: np.ndarray,
    test_size: float,
    seed: int,
    cv_folds: int,
) -> tuple[GridSearchCV, float, float]:
    x_train, x_test, y_train, y_test = train_test_split(
        x,
        y,
        test_size=test_size,
        random_state=seed,
        stratify=y,
    )

    param_grid = {
        "mlp__hidden_layer_sizes": [(40, 20), (64, 32), (128, 64), (128, 128)],
        "mlp__alpha": [1e-5, 1e-4, 1e-3],
        "mlp__learning_rate_init": [3e-4, 1e-3, 3e-3],
    }
    search = GridSearchCV(
        estimator=build_model(random_state=seed),
        param_grid=param_grid,
        scoring="f1_macro",
        cv=StratifiedKFold(n_splits=cv_folds, shuffle=True, random_state=seed),
        n_jobs=-1,
        refit=True,
    )
    search.fit(x_train, y_train)

    y_pred = search.best_estimator_.predict(x_test)
    return search, accuracy_score(y_test, y_pred), f1_score(y_test, y_pred, average="macro")


def save_selection_plot(search: GridSearchCV, output_path: Path) -> None:
    results = search.cv_results_
    architectures = [(40, 20), (64, 32), (128, 64), (128, 128)]
    best_scores = []

    for architecture in architectures:
        indices = [
            index
            for index, params in enumerate(results["params"])
            if params["mlp__hidden_layer_sizes"] == architecture
        ]
        best_scores.append(max(results["mean_test_score"][index] for index in indices))

    labels = [f"{a}-{b}" for a, b in architectures]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    bars = ax.bar(labels, best_scores, color="#59A14F", edgecolor="#274b24", linewidth=0.8)
    ax.set_xlabel("Neuronas en capas ocultas")
    ax.set_ylabel("F1 macro medio en validación cruzada")
    ax.set_ylim(max(0.0, min(best_scores) - 0.08), 1.01)
    ax.grid(axis="y", alpha=0.25)

    for bar, score in zip(bars, best_scores):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            score + 0.004,
            f"{score:.3f}",
            ha="center",
            va="bottom",
        )

    best_architecture = search.best_params_["mlp__hidden_layer_sizes"]
    best_label = f"{best_architecture[0]}-{best_architecture[1]}"
    best_index = labels.index(best_label)
    bars[best_index].set_color("#F28E2B")
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def save_architecture_plot(
    best_params: dict[str, object],
    best_cv_f1: float,
    test_accuracy: float,
    test_macro_f1: float,
    output_path: Path,
) -> None:
    hidden_layers = best_params["mlp__hidden_layer_sizes"]
    layer_sizes = [2, *hidden_layers, 9]
    layer_names = ["Entrada", "Oculta 1", "Oculta 2", "Salida"]
    parameter_counts = [
        (layer_sizes[index] + 1) * layer_sizes[index + 1] for index in range(len(layer_sizes) - 1)
    ]

    fig, ax = plt.subplots(figsize=(9.0, 4.6))
    ax.axis("off")

    x_positions = np.linspace(0.08, 0.78, len(layer_sizes))
    max_size = max(layer_sizes)
    for index, (x_position, size, name) in enumerate(zip(x_positions, layer_sizes, layer_names)):
        height = 0.18 + 0.46 * size / max_size
        rect = plt.Rectangle(
            (x_position, 0.5 - height / 2),
            0.13,
            height,
            facecolor="#E8F1FA",
            edgecolor="#26384f",
            linewidth=1.2,
        )
        ax.add_patch(rect)
        ax.text(x_position + 0.065, 0.5, f"{size}", ha="center", va="center", fontsize=16)
        ax.text(x_position + 0.065, 0.19, name, ha="center", va="center")

        if index < len(layer_sizes) - 1:
            ax.annotate(
                "",
                xy=(x_positions[index + 1], 0.5),
                xytext=(x_position + 0.13, 0.5),
                arrowprops={"arrowstyle": "->", "color": "#26384f", "linewidth": 1.2},
            )
            ax.text(
                (x_position + x_positions[index + 1] + 0.13) / 2,
                0.62,
                f"{parameter_counts[index]} par.",
                ha="center",
                va="center",
                fontsize=9,
            )

    total_parameters = sum(parameter_counts)
    summary = (
        f"Arquitectura seleccionada: 2-{hidden_layers[0]}-{hidden_layers[1]}-9\n"
        f"Parámetros entrenables: {total_parameters}\n"
        f"α={best_params['mlp__alpha']:.0e} · lr={best_params['mlp__learning_rate_init']:.0e}\n"
        f"F1 macro CV: {best_cv_f1:.3f}\n"
        f"Test: acc={test_accuracy:.3f}, F1={test_macro_f1:.3f}"
    )
    ax.text(
        0.86,
        0.5,
        summary,
        ha="left",
        va="center",
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#F7F7F7", "edgecolor": "#B0B0B0"},
    )

    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    default_output_dir = script_dir.parents[1] / "TFM_Guante_Haptico" / "Imagenes"

    parser = argparse.ArgumentParser(description="Generate glove flexion figures for the TFM")
    parser.add_argument(
        "--dataset",
        type=Path,
        default=script_dir / "dataset_glove_2in_9pos_new.txt",
        help="Path to the dataset txt file",
    )
    parser.add_argument("--output-dir", type=Path, default=default_output_dir, help="Figure output folder")
    parser.add_argument("--test-size", type=float, default=0.2, help="Test split ratio")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--cv-folds", type=int, default=5, help="Stratified CV folds")
    args = parser.parse_args()

    x, y = load_dataset(args.dataset)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    histogram_path = args.output_dir / "histograma_dataset_flexion.png"
    boxplot_path = args.output_dir / "boxplot_dataset_flexion.png"
    selection_path = args.output_dir / "seleccion_arquitectura_fcnn.png"
    architecture_path = args.output_dir / "arquitectura_red_flexion.png"

    save_histogram(y, histogram_path)
    save_boxplot(x, y, boxplot_path)

    search, test_accuracy, test_macro_f1 = run_architecture_search(
        x=x,
        y=y,
        test_size=args.test_size,
        seed=args.seed,
        cv_folds=args.cv_folds,
    )
    save_selection_plot(search, selection_path)
    save_architecture_plot(
        best_params=search.best_params_,
        best_cv_f1=float(search.best_score_),
        test_accuracy=float(test_accuracy),
        test_macro_f1=float(test_macro_f1),
        output_path=architecture_path,
    )

    labels, counts = np.unique(y, return_counts=True)
    print(f"Dataset: {args.dataset}")
    print(f"Samples: {len(y)}")
    print("Samples per position:", dict(zip(labels.astype(int), counts.astype(int))))
    print(f"Best CV macro-F1: {search.best_score_:.4f}")
    print(f"Best params: {search.best_params_}")
    print(f"Test accuracy: {test_accuracy:.4f}")
    print(f"Test macro-F1: {test_macro_f1:.4f}")
    print("Generated figures:")
    for path in [histogram_path, boxplot_path, selection_path, architecture_path]:
        print(f"  {path}")


if __name__ == "__main__":
    main()
