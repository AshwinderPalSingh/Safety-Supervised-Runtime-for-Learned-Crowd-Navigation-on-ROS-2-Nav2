#!/usr/bin/env python3
"""Plots from episodes.csv/interventions.csv (IMPLEMENTATION_PLAN.md S4.9) - variance/
distributions, not just means, per the phase's own done-bar wording. Reads CSVs written by
run_matrix.py; does not compute anything during the run itself.
"""
import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

CAUSE_NAMES = [
    'CROWD_SIZE', 'PROXIMITY', 'RELATIVE_SPEED', 'COMMAND_LIMIT', 'LOW_PERCEPTION_CONFIDENCE',
    'COSTMAP_COLLISION', 'KEEPOUT_VIOLATION', 'INFERENCE_TIMEOUT',
]


def plot_outcome_rates(df, out_dir):
    core = df[~df['episode_id'].str.startswith(('pilot_', 'noise_sweep_', 'depot_keepout_block'))]
    if core.empty:
        return
    grouped = core.groupby(['scenario_name', 'config_name'])['outcome'].value_counts(
        normalize=True).unstack(fill_value=0.0)
    fig, ax = plt.subplots(figsize=(10, 5))
    grouped.plot(kind='bar', stacked=True, ax=ax)
    ax.set_ylabel('fraction of episodes')
    ax.set_title('Episode outcome rate by scenario family x config (N=%d/cell)' %
                 core.groupby(['scenario_name', 'config_name']).size().min())
    ax.legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'outcome_rates.png'), dpi=150)
    plt.close(fig)


def plot_efficiency_distributions(df, out_dir):
    core = df[~df['episode_id'].str.startswith(('pilot_', 'noise_sweep_', 'depot_keepout_block'))]
    core = core[core['outcome'] == 'success']
    if core.empty:
        return
    for metric in ('duration_s', 'path_length_m'):
        fig, ax = plt.subplots(figsize=(10, 5))
        data = []
        labels = []
        for (scenario, config), sub in core.groupby(['scenario_name', 'config_name']):
            data.append(sub[metric].dropna().values)
            labels.append(f"{scenario}\n{config}")
        ax.boxplot(data, tick_labels=labels, showmeans=True)
        ax.set_ylabel(metric)
        ax.set_title(f'{metric} distribution, successful episodes only')
        plt.xticks(rotation=45, ha='right')
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f'{metric}_distribution.png'), dpi=150)
        plt.close(fig)


def plot_intervention_rate_by_family(df, out_dir):
    """The headline number (S4.9.4): intervention rate broken down by cause, compared across
    scenario families - not a footnote to the success/collision table."""
    core = df[~df['episode_id'].str.startswith(('pilot_', 'noise_sweep_', 'depot_keepout_block'))]
    core = core[core['config_name'] == 'policy_supervised']
    if core.empty:
        return
    cause_cols = [f'intervention_count_cause_{c}' for c in range(8)]
    per_family = core.groupby('scenario_name')[cause_cols].sum()
    n_episodes = core.groupby('scenario_name').size()
    rate = per_family.div(n_episodes, axis=0)
    rate.columns = CAUSE_NAMES
    fig, ax = plt.subplots(figsize=(10, 5))
    rate.T.plot(kind='bar', ax=ax)
    ax.set_ylabel('interventions per episode')
    ax.set_title('Intervention rate by cause: open_arena vs depot (policy_supervised)')
    plt.xticks(rotation=45, ha='right')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'intervention_rate_by_family.png'), dpi=150)
    plt.close(fig)


def plot_noise_sweep(df, out_dir):
    sweep = df[df['episode_id'].str.startswith('noise_sweep_')]
    if sweep.empty:
        return
    grouped = sweep.groupby('dropout_prob').agg(
        low_perception_rate=('intervention_count_cause_4', 'mean'),
        success_rate=('outcome', lambda s: (s == 'success').mean()),
    ).reset_index()
    fig, ax1 = plt.subplots(figsize=(8, 5))
    ax1.plot(grouped['dropout_prob'], grouped['low_perception_rate'], 'o-', color='tab:red',
             label='mean LOW_PERCEPTION_CONFIDENCE interventions/episode')
    ax1.set_xlabel('perception dropout_prob')
    ax1.set_ylabel('mean interventions/episode', color='tab:red')
    ax2 = ax1.twinx()
    ax2.plot(grouped['dropout_prob'], grouped['success_rate'], 's-', color='tab:blue',
             label='success rate')
    ax2.set_ylabel('success rate', color='tab:blue')
    fig.suptitle('Perception noise sweep (policy_supervised / open_arena / reactive)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'noise_sweep.png'), dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir', required=True)
    args = parser.parse_args()
    episodes_path = os.path.join(args.results_dir, 'episodes.csv')
    df = pd.read_csv(episodes_path)
    out_dir = os.path.join(args.results_dir, 'plots')
    os.makedirs(out_dir, exist_ok=True)

    plot_outcome_rates(df, out_dir)
    plot_efficiency_distributions(df, out_dir)
    plot_intervention_rate_by_family(df, out_dir)
    plot_noise_sweep(df, out_dir)
    print(f"Wrote plots to {out_dir}")


if __name__ == '__main__':
    main()
