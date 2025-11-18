#!/usr/bin/env python3
"""
Thruster Data Analysis Tool
Analyzes CSV data from MPC_Optimized_Thrust_Test.py
Provides comprehensive thrust characterization for MPC controller tuning
Now supports multiple tests per thruster and cross-thruster comparison
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import glob
import shutil
import re
from datetime import datetime
import seaborn as sns
from collections import defaultdict
from typing import Dict, List, Optional, Any, Tuple

class ThrusterDataAnalyzer:
    """Comprehensive analysis of thruster calibration data with steady-state focus"""

    def __init__(self, csv_file_path: str):
        """Initialize analyzer with CSV file path"""
        self.csv_file = pathlib.Path(csv_file_path)
        self.df = None
        self.results = {}
        self.summary_stats = {}
        self.output_dir = None
        self.thruster_number = None
        self.steady_state_start_offset = 3.0  # Ignore first 3 seconds
        self.steady_state_end_offset = 1.0    # Ignore last 1 second

        self._extract_thruster_number()

        # Create organized output directory
        self._create_output_directory()

    def _extract_thruster_number(self):
        """Extract thruster number from filename"""
        filename = self.csv_file.name
        patterns = [
            r'Thruster\((\d+)\)',
            r'thruster_(\d+)',
            r'Thruster(\d+)',
        ]

        for pattern in patterns:
            match = re.search(pattern, filename, re.IGNORECASE)
            if match:
                self.thruster_number = int(match.group(1))
                break

        if self.thruster_number is None:
            self.thruster_number = 1
            print(f"Could not extract thruster number from filename, defaulting to 1")

    def _create_output_directory(self):
        """Create organized output directory structure"""
        base_dir = self.csv_file.parent
        data_folder_name = f"Thruster{self.thruster_number}_data"
        data_dir = base_dir / data_folder_name
        data_dir.mkdir(exist_ok=True)

        analysis_folder_name = f"Thruster{self.thruster_number}_analysis"
        self.output_dir = base_dir / analysis_folder_name
        self.output_dir.mkdir(exist_ok=True)

        print(f" Created analysis directory: {self.output_dir}")

        # Copy the original CSV file to the data directory
        destination_csv = data_dir / self.csv_file.name
        shutil.copy2(self.csv_file, destination_csv)
        print(f"Copied source data to: {destination_csv}")

    def load_and_clean_data(self) -> pd.DataFrame:
        """Load CSV data and clean it"""
        print(f"Loading data from: {self.csv_file}")

        # Load data
        self.df = pd.read_csv(self.csv_file)

        # Basic info
        print(f"Loaded {len(self.df)} data points")
        print(f"Columns: {list(self.df.columns)}")

        # Clean data
        initial_rows = len(self.df)
        self.df = self.df.dropna(subset=['weight_g'])  # Remove empty weight readings
        self.df['weight_g'] = pd.to_numeric(self.df['weight_g'], errors='coerce')
        self.df = self.df.dropna(subset=['weight_g'])  # Remove any conversion failures

        print(f"After cleaning: {len(self.df)} data points ({len(self.df)/initial_rows * 100:.1f}% retained)")

        return self.df

    def analyze_individual_pulses(self) -> Dict:
        """Analyze each pulse individually"""
        print("\nAnalyzing individual pulses...")

        results = {}  # type: ignore[index]
        pulse_numbers = sorted([p for p in self.df['pulse_number'].unique() if p > 0])  # type: ignore[index]

        print(f"Found {len(pulse_numbers)} pulses to analyze")

        for pulse_num in pulse_numbers:  # type: ignore[index]
            pulse_subset = self.df[self.df['pulse_number'] == pulse_num]  # type: ignore[index]

            # Get different phases
            pre_pulse_data = pulse_subset[pulse_subset['phase'] == 'pre_pulse']
            firing_data = pulse_subset[pulse_subset['phase'] == 'firing']
            post_pulse_data = pulse_subset[pulse_subset['phase'] == 'post_pulse']

            if len(pre_pulse_data) > 0 and len(firing_data) > 0:
                baseline_weight = np.median(pre_pulse_data['weight_g'])

                # Filter firing data to exclude first 1.0s and last 0.5s of pulse
                firing_times = firing_data['t_s'].values
                pulse_start_time = firing_times[0]
                pulse_end_time = firing_times[-1]
                pulse_duration = pulse_end_time - pulse_start_time

                if pulse_duration >= 2.0:
                    steady_start_time = pulse_start_time + 1.0
                    steady_end_time = pulse_end_time - 0.5

                    # Filter to steady-state data only
                    steady_mask = (firing_data['t_s'] >= steady_start_time) & (firing_data['t_s'] <= steady_end_time)
                    steady_firing_data = firing_data[steady_mask]

                    if len(steady_firing_data) > 5:  # Need sufficient steady-state data points
                        # Calculate thrust values using only steady-state data
                        thrust_values = steady_firing_data['weight_g'] - baseline_weight

                        positive_thrust = thrust_values[thrust_values > 0]
                else:
                    # Pulse too short, skip analysis
                    positive_thrust = np.array([])
  # type: ignore[possibly-unbound]
                if len(positive_thrust) > 5:  # Need sufficient data points  # type: ignore[possibly-unbound]
                    # Calculate various thrust metrics
                    results[pulse_num] = {'baseline_weight': baseline_weight,  # type: ignore[possibly-unbound]
                        'peak_thrust': np.max(positive_thrust),  # type: ignore[possibly-unbound]
                        'mean_thrust': np.mean(positive_thrust),  # type: ignore[possibly-unbound]
                        'median_thrust': np.median(positive_thrust),  # type: ignore[possibly-unbound]
                        'thrust_std': np.std(positive_thrust),  # type: ignore[possibly-unbound]
                        'thrust_75th': np.percentile(positive_thrust, 75),  # 75th percentile  # type: ignore[possibly-unbound]
                        'thrust_90th': np.percentile(positive_thrust, 90),  # 90th percentile  # type: ignore[possibly-unbound]
                        'thrust_consistency_cv': np.std(positive_thrust) / np.mean(positive_thrust) * 100,  # type: ignore[possibly-unbound]
                        'pulse_duration': pulse_duration,  # Total pulse duration  # type: ignore[possibly-unbound,operator]
                        'steady_state_duration': steady_end_time - steady_start_time,  # Steady-state analysis window  # type: ignore[possibly-unbound]
                        'data_points': len(positive_thrust),  # type: ignore[possibly-unbound]
                        'firing_start_time': firing_data['t_s'].iloc[0] if len(firing_data) > 0 else None,
                        'firing_end_time': firing_data['t_s'].iloc[-1] if len(firing_data) > 0 else None,  # type: ignore[possibly-unbound]
                        'steady_start_time': steady_start_time,  # type: ignore[possibly-unbound]
                        'steady_end_time': steady_end_time,  # type: ignore[possibly-unbound]
                    }

                    print(f"  Pulse {pulse_num}: {results[pulse_num]['mean_thrust']:.2f}g ± {results[pulse_num]['thrust_std']:.2f}g (steady-state: {results[pulse_num]['steady_state_duration']:.1f}s)")

        self.results = results
        return results

    def calculate_summary_statistics(self) -> Dict:
        """Calculate overall summary statistics"""
        if not self.results:
            print("No pulse results available. Run analyze_individual_pulses() first.")
            return {}

        print("\nCalculating summary statistics...")

        # Extract metrics across all pulses
        all_mean_thrusts = [self.results[p]['mean_thrust'] for p in self.results.keys()]
        all_peak_thrusts = [self.results[p]['peak_thrust'] for p in self.results.keys()]
        all_median_thrusts = [self.results[p]['median_thrust'] for p in self.results.keys()]
        all_cv_values = [self.results[p]['thrust_consistency_cv'] for p in self.results.keys()]

        self.summary_stats = {'num_pulses': len(all_mean_thrusts),
            'overall_mean_thrust': np.mean(all_mean_thrusts),
            'overall_std_thrust': np.std(all_mean_thrusts),
            'overall_median_thrust': np.median(all_median_thrusts),
            'overall_peak_thrust': np.max(all_peak_thrusts),
            'thrust_repeatability_cv': np.std(all_mean_thrusts) / np.mean(all_mean_thrusts) * 100,
            'min_thrust': np.min(all_mean_thrusts),
            'max_thrust': np.max(all_mean_thrusts),
            'thrust_range': np.max(all_mean_thrusts) - np.min(all_mean_thrusts),
            'average_pulse_consistency': np.mean(all_cv_values),
            'recommended_thrust_value': np.median(all_median_thrusts),  # Most reliable estimate
            'recommended_thrust_N': np.median(all_median_thrusts) * 9.81 / 1000,  # Convert to Newtons
        }

        return self.summary_stats

    def create_comprehensive_plots(self) -> None:
        """Create comprehensive visualization dashboard"""
        print("\nCreating visualization dashboard...")

        # Set up the plotting style
        plt.style.use('default')
        sns.set_palette("husl")

        fig = plt.figure(figsize=(20, 16))
        gs = fig.add_gridspec(4, 3, height_ratios=[1, 1, 1, 1], width_ratios=[1, 1, 1])

        # Plot 1: Raw time series data
        ax1 = fig.add_subplot(gs[0, :])
        phase_colors = {'baseline': 'lightblue', 'pre_pulse': 'orange', 'firing': 'red', 'post_pulse': 'green', 'pause': 'gray'}

        for phase in phase_colors:  # type: ignore[index]
            phase_data = self.df[self.df['phase'] == phase]  # type: ignore[index]
            if len(phase_data) > 0:
                ax1.scatter(phase_data['t_s'], phase_data['weight_g'],
                           c=phase_colors[phase], label=phase, alpha=0.6, s=15)

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Weight Reading (g)')
        ax1.set_title(f'Raw Thrust Data Over Time - {self.csv_file.name}', fontsize=14, fontweight='bold')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)

        # Plot 2: Individual pulse thrust values
        ax2 = fig.add_subplot(gs[1, 0])
        pulse_numbers = list(self.results.keys())
        mean_thrusts = [self.results[p]['mean_thrust'] for p in pulse_numbers]
        thrust_stds = [self.results[p]['thrust_std'] for p in pulse_numbers]

        ax2.errorbar(pulse_numbers, mean_thrusts, yerr=thrust_stds,
                    fmt='o-', capsize=5, capthick=2, linewidth=2, markersize=6)
        ax2.set_xlabel('Pulse Number')
        ax2.set_ylabel('Mean Thrust (g)')
        ax2.set_title('Thrust Consistency Across Pulses')
        ax2.grid(True, alpha=0.3)

        # Plot 3: Thrust distribution histogram
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.hist(mean_thrusts, bins=min(15, len(mean_thrusts)//2), alpha=0.7, edgecolor='black', color='skyblue')  # type: ignore[arg-type]
        ax3.axvline(np.mean(mean_thrusts), color='red', linestyle='--', linewidth=2,  # type: ignore[arg-type]
                    label=f'Mean: {np.mean(mean_thrusts):.2f}g')  # type: ignore[arg-type]
        ax3.axvline(np.median(mean_thrusts), color='green', linestyle='--', linewidth=2,  # type: ignore[arg-type]
                    label=f'Median: {np.median(mean_thrusts):.2f}g')
        ax3.set_xlabel('Mean Thrust (g)')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Thrust Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Plot 4: Thrust metrics comparison
        ax4 = fig.add_subplot(gs[1, 2])
        metrics = ['peak_thrust', 'thrust_90th', 'mean_thrust', 'median_thrust', 'thrust_75th']
        metric_labels = ['Peak', '90th %ile', 'Mean', 'Median', '75th %ile']
        metric_values = {metric: [self.results[p][metric] for p in pulse_numbers] for metric in metrics}

        means = [np.mean(metric_values[m]) for m in metrics]
        stds = [np.std(metric_values[m]) for m in metrics]

        bars = ax4.bar(range(len(means)), means, yerr=stds, capsize=5, alpha=0.7,
                      color=['red', 'orange', 'blue', 'green', 'purple'])
        ax4.set_xticks(range(len(means)))
        ax4.set_xticklabels(metric_labels, rotation=45)
        ax4.set_ylabel('Thrust (g)')
        ax4.set_title('Thrust Metrics Comparison')
        ax4.grid(True, alpha=0.3)

        # Add value labels on bars
        for bar, mean_val in zip(bars, means):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + np.max(stds)*0.1,
                    f'{mean_val:.1f}', ha='center', va='bottom', fontweight='bold')

        # Plot 5: Thrust consistency over time
        ax5 = fig.add_subplot(gs[2, 0])
        consistency_values = [self.results[p]['thrust_consistency_cv'] for p in pulse_numbers]
        ax5.plot(pulse_numbers, consistency_values, 'o-', linewidth=2, markersize=6, color='purple')
        ax5.set_xlabel('Pulse Number')
        ax5.set_ylabel('Coefficient of Variation (%)')
        ax5.set_title('Thrust Consistency (Lower = Better)')
        ax5.grid(True, alpha=0.3)

        # Plot 6: Detailed view of a single pulse with steady-state highlighting
        ax6 = fig.add_subplot(gs[2, 1])
        if pulse_numbers:
            sample_pulse = pulse_numbers[len(pulse_numbers)//2]  # Middle pulse  # type: ignore[index]
            sample_data = self.df[self.df['pulse_number'] == sample_pulse]  # type: ignore[index]

            for phase in ['pre_pulse', 'firing', 'post_pulse']:
                phase_data = sample_data[sample_data['phase'] == phase]
                if len(phase_data) > 0:
                    ax6.plot(phase_data['t_s'], phase_data['weight_g'],
                            'o-', label=phase, linewidth=2, markersize=4)

            # Highlight steady-state analysis window
            if sample_pulse in self.results:
                steady_start = self.results[sample_pulse]['steady_start_time']
                steady_end = self.results[sample_pulse]['steady_end_time']
                ax6.axvspan(steady_start, steady_end, alpha=0.3, color='blue',
                           label=f'Steady-state analysis\n({steady_end-steady_start:.1f}s)')

            ax6.set_xlabel('Time (s)')
            ax6.set_ylabel('Weight (g)')
            ax6.set_title(f'Detailed View - Pulse {sample_pulse}\n(Blue = Analysis Window)')
            ax6.legend()
            ax6.grid(True, alpha=0.3)

        # Plot 7: Performance summary as text
        ax7 = fig.add_subplot(gs[2, 2])
        ax7.axis('off')

        summary_text = f"""
THRUSTER {self.thruster_number} PERFORMANCE SUMMARY
(Steady-State Analysis: First {self.steady_state_start_offset}s & Last {self.steady_state_end_offset}s Excluded)

Number of Pulses: {self.summary_stats['num_pulses']}
Recommended Thrust: {self.summary_stats['recommended_thrust_value']:.2f} g
                   ({self.summary_stats['recommended_thrust_N']:.6f} N)

Mean Thrust: {self.summary_stats['overall_mean_thrust']:.2f} ± {self.summary_stats['overall_std_thrust']:.2f} g
Peak Thrust: {self.summary_stats['overall_peak_thrust']:.2f} g
Thrust Range: {self.summary_stats['min_thrust']:.2f} - {self.summary_stats['max_thrust']:.2f} g

Repeatability: {self.summary_stats['thrust_repeatability_cv']:.1f}% CV
Avg Consistency: {self.summary_stats['average_pulse_consistency']:.1f}% CV

ANALYSIS METHOD:
- Excludes first {self.steady_state_start_offset:.1f}s (startup transients)
- Excludes last {self.steady_state_end_offset:.1f}s (shutdown transients)
- Uses steady-state data only

RECOMMENDATION FOR MPC:
Use {self.summary_stats['recommended_thrust_N']:.6f} N
in SET_EFFECTS matrix for Thruster {self.thruster_number}
        """

        ax7.text(0.05, 0.95, summary_text, transform=ax7.transAxes, fontsize=11,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))

        # Plot 8: Box plot comparison
        ax8 = fig.add_subplot(gs[3, 0])
        thrust_data_for_box = [metric_values[m] for m in metrics]  # type: ignore[call-arg]
        box_plot = ax8.boxplot(thrust_data_for_box, labels=metric_labels, patch_artist=True)  # type: ignore[call-arg]

        colors = ['red', 'orange', 'blue', 'green', 'purple']
        for patch, color in zip(box_plot['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        ax8.set_ylabel('Thrust (g)')
        ax8.set_title('Thrust Metrics Distribution')
        ax8.tick_params(axis='x', rotation=45)
        ax8.grid(True, alpha=0.3)

        # Plot 9: Cumulative thrust analysis
        ax9 = fig.add_subplot(gs[3, 1])
        sorted_thrusts = np.sort(mean_thrusts)
        cumulative_prob = np.arange(1, len(sorted_thrusts) + 1) / len(sorted_thrusts)
        ax9.plot(sorted_thrusts, cumulative_prob * 100, 'b-', linewidth=2)
        ax9.axvline(self.summary_stats['recommended_thrust_value'], color='red', linestyle='--',
                   label=f'Recommended: {self.summary_stats["recommended_thrust_value"]:.2f}g')
        ax9.set_xlabel('Thrust (g)')
        ax9.set_ylabel('Cumulative Probability (%)')
        ax9.set_title('Cumulative Thrust Distribution')
        ax9.legend()
        ax9.grid(True, alpha=0.3)

        # Plot 10: Thrust vs time correlation
        ax10 = fig.add_subplot(gs[3, 2])
        start_times = [self.results[p]['firing_start_time'] for p in pulse_numbers if self.results[p]['firing_start_time']]
        if start_times:
            ax10.scatter(start_times, mean_thrusts, alpha=0.7, s=50)

            # Add trend line
            z = np.polyfit(start_times, mean_thrusts, 1)
            p = np.poly1d(z)
            ax10.plot(start_times, p(start_times), "r--", alpha=0.8, linewidth=2)

            ax10.set_xlabel('Pulse Start Time (s)')
            ax10.set_ylabel('Mean Thrust (g)')
            ax10.set_title('Thrust vs Time Correlation')
            ax10.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save the plot to the organized output directory
        plot_filename = f"thruster_analysis_{self.csv_file.stem}.png"  # type: ignore[operator]
        plot_path = self.output_dir / plot_filename  # type: ignore[operator]
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Analysis plot saved to: {plot_path}")

    def print_detailed_results(self) -> None:
        """Print detailed analysis results"""
        print("\n" + "="*60)
        print("DETAILED THRUSTER ANALYSIS RESULTS")
        print("="*60)

        print(f"\nFile analyzed: {self.csv_file.name}")
        print(f"Analysis timestamp: {pd.Timestamp.now()}")

        print(f"\nOVERALL STATISTICS:")
        for key, value in self.summary_stats.items():
            if 'thrust' in key.lower():
                if key.endswith('_N'):
                    print(f"  {key.replace('_', ' ').title()}: {value:.6f}")
                else:
                    print(f"  {key.replace('_', ' ').title()}: {value:.3f}")
            else:
                print(f"  {key.replace('_', ' ').title()}: {value:.3f}")

        print(f"\nRECOMMENDED VALUES FOR MPC:")
        print(f"  Thrust for SET_EFFECTS matrix: {self.summary_stats['recommended_thrust_N']:.6f} N")
        print(f"  Equivalent in grams-force: {self.summary_stats['recommended_thrust_value']:.3f} g")

        print(f"\nINDIVIDUAL PULSE ANALYSIS (Steady-State Only):")
        print(f"Excludes first {self.steady_state_start_offset}s and last {self.steady_state_end_offset}s of each pulse")
        print("Pulse | Mean (g) | Peak (g) | Std (g) | CV (%) | SS Duration (s)")
        print("-" * 70)
        for pulse_num in sorted(self.results.keys()):
            r = self.results[pulse_num]
            ss_duration = r.get('steady_state_duration', r['pulse_duration'])
            print(f"{pulse_num:5d} | {r['mean_thrust']:8.2f} | {r['peak_thrust']:8.2f} | "
                  f"{r['thrust_std']:7.2f} | {r['thrust_consistency_cv']:6.1f} | {ss_duration:14.2f}")

    def save_results_to_file(self) -> None:
        """Save analysis results to text file in organized directory"""
        results_filename = f"analysis_results_{self.csv_file.stem}.txt"  # type: ignore[operator]
        results_path = self.output_dir / results_filename  # type: ignore[operator]

        with open(results_path, 'w') as f:
            f.write("THRUSTER DATA ANALYSIS RESULTS\n")
            f.write("="*50 + "\n\n")
            f.write(f"Source file: {self.csv_file.name}\n")
            f.write(f"Analysis date: {pd.Timestamp.now()}\n")
            f.write(f"Thruster number: {self.thruster_number}\n")
            f.write(f"Analysis method: Steady-state (excludes first {self.steady_state_start_offset}s and last {self.steady_state_end_offset}s)\n\n")

            f.write("SUMMARY STATISTICS:\n")
            f.write("-"*30 + "\n")
            for key, value in self.summary_stats.items():
                f.write(f"{key}: {value}\n")

            f.write(f"\nRECOMMENDED MPC VALUES:\n")
            f.write("-"*30 + "\n")
            f.write(f"Thrust (Newtons): {self.summary_stats['recommended_thrust_N']:.6f}\n")
            f.write(f"Thrust (grams): {self.summary_stats['recommended_thrust_value']:.3f}\n")

            f.write(f"\nINDIVIDUAL PULSE DATA:\n")
            f.write("-"*30 + "\n")
            f.write("Pulse,Mean_Thrust_g,Peak_Thrust_g,Std_g,CV_percent,Total_Duration_s,SteadyState_Duration_s\n")
            for pulse_num in sorted(self.results.keys()):
                r = self.results[pulse_num]
                ss_duration = r.get('steady_state_duration', r['pulse_duration'])
                f.write(f"{pulse_num},{r['mean_thrust']:.3f},{r['peak_thrust']:.3f},"
                       f"{r['thrust_std']:.3f},{r['thrust_consistency_cv']:.1f},"
                       f"{r['pulse_duration']:.2f},{ss_duration:.2f}\n")

        print(f"Results saved to: {results_path}")
  # type: ignore[operator]
        mpc_summary_path = self.output_dir / f"MPC_Summary_Thruster{self.thruster_number}.txt"  # type: ignore[operator]
        with open(mpc_summary_path, 'w') as f:
            f.write(f"MPC THRUSTER {self.thruster_number} CHARACTERIZATION\n")
            f.write("="*50 + "\n\n")
            f.write(f"Recommended thrust value for SET_EFFECTS matrix:\n")
            f.write(f"Thruster {self.thruster_number}: {self.summary_stats['recommended_thrust_N']:.6f} N\n\n")
            f.write(f"Performance metrics:\n")
            f.write(f"- Repeatability: {self.summary_stats['thrust_repeatability_cv']:.1f}% CV\n")
            f.write(f"- Number of test pulses: {self.summary_stats['num_pulses']}\n")
            f.write(f"- Analysis method: Steady-state (+{self.steady_state_start_offset}s to -{self.steady_state_end_offset}s)\n")

        print(f"MPC summary saved to: {mpc_summary_path}")

    def run_complete_analysis(self) -> None:
        """Run the complete analysis pipeline"""
        print("Starting complete thruster data analysis...")

        # Load and clean data
        self.load_and_clean_data()

        # Analyze individual pulses
        self.analyze_individual_pulses()

        # Calculate summary statistics
        self.calculate_summary_statistics()

        # Create visualizations
        self.create_comprehensive_plots()

        # Print results
        self.print_detailed_results()

        # Save results
        self.save_results_to_file()

        print("\n Analysis complete!")


class MultiTestThrusterAnalyzer:
    """Analyzer for multiple tests of the same thruster"""

    def __init__(self, thruster_number: int, test_files: List[str]):
        """Initialize with thruster number and list of test files"""
        self.thruster_number = thruster_number
        self.test_files = [pathlib.Path(f) for f in test_files]
        self.test_results = {}
        self.combined_stats = {}
        self.output_dir = None
        self._create_output_directory()

    def _create_output_directory(self):
        """Create output directory for multi-test analysis"""
        folder_name = f"Thruster{self.thruster_number}_analysis"

        # Create directory in the same folder as the test files
        base_dir = self.test_files[0].parent
        self.output_dir = base_dir / folder_name
        self.output_dir.mkdir(exist_ok=True)

        print(f" Created multi-test analysis directory: {self.output_dir}")

    def analyze_all_tests(self) -> Dict:
        """Analyze all test files for this thruster"""
        print(f"\nAnalyzing {len(self.test_files)} tests for Thruster {self.thruster_number}")

        for i, test_file in enumerate(self.test_files, 1):
            print(f"\nAnalyzing Test {i}: {test_file.name}")

            try:
                analyzer = ThrusterDataAnalyzer(str(test_file))
                analyzer.load_and_clean_data()
                analyzer.analyze_individual_pulses()
                analyzer.calculate_summary_statistics()

                self.test_results[f"Test_{i}"] = {'analyzer': analyzer,
                    'filename': test_file.name,
                    'summary_stats': analyzer.summary_stats,
                    'pulse_results': analyzer.results
                }

                print(f"   Test {i} complete: {analyzer.summary_stats['recommended_thrust_value']:.3f}g")

            except Exception as e:
                print(f"   Error analyzing Test {i}: {e}")
                continue

        return self.test_results

    def calculate_combined_statistics(self) -> Dict:
        """Calculate statistics across all tests"""
        print(f"\nCalculating combined statistics for Thruster {self.thruster_number}")

        if not self.test_results:
            print("No test results available")
            return {}

        all_recommended_thrusts = []
        all_mean_thrusts = []
        all_repeatability_cvs = []
        all_pulse_counts = []

        for test_name, test_data in self.test_results.items():
            stats = test_data['summary_stats']
            all_recommended_thrusts.append(stats['recommended_thrust_value'])
            all_mean_thrusts.append(stats['overall_mean_thrust'])
            all_repeatability_cvs.append(stats['thrust_repeatability_cv'])
            all_pulse_counts.append(stats['num_pulses'])

        # Calculate inter-test statistics
        self.combined_stats = {'thruster_number': self.thruster_number,
            'num_tests': len(self.test_results),
            'total_pulses': sum(all_pulse_counts),

            # Primary thrust metrics
            'mean_recommended_thrust': np.mean(all_recommended_thrusts),
            'std_recommended_thrust': np.std(all_recommended_thrusts),
            'cv_recommended_thrust': np.std(all_recommended_thrusts) / np.mean(all_recommended_thrusts) * 100,

            # Overall thrust metrics
            'mean_overall_thrust': np.mean(all_mean_thrusts),
            'std_overall_thrust': np.std(all_mean_thrusts),
            'cv_overall_thrust': np.std(all_mean_thrusts) / np.mean(all_mean_thrusts) * 100,

            # Test-to-test consistency
            'min_recommended_thrust': np.min(all_recommended_thrusts),
            'max_recommended_thrust': np.max(all_recommended_thrusts),
            'thrust_range': np.max(all_recommended_thrusts) - np.min(all_recommended_thrusts),

            # Repeatability metrics
            'average_repeatability_cv': np.mean(all_repeatability_cvs),
            'test_to_test_consistency': np.std(all_recommended_thrusts) / np.mean(all_recommended_thrusts) * 100,

            # Final recommendation
            'final_recommended_thrust_g': np.median(all_recommended_thrusts),
            'final_recommended_thrust_N': np.median(all_recommended_thrusts) * 9.81 / 1000,

            'individual_test_thrusts': all_recommended_thrusts,
            'individual_test_names': list(self.test_results.keys())
        };

        return self.combined_stats

    def create_multi_test_plots(self) -> None:
        """Create comprehensive plots comparing multiple tests"""
        print(f"\nCreating multi-test visualization for Thruster {self.thruster_number}")

        if len(self.test_results) < 2:
            print("Need at least 2 tests for comparison plots")
            return

        # Set up plotting
        plt.style.use('default')
        fig = plt.figure(figsize=(20, 14))
        gs = fig.add_gridspec(4, 3, height_ratios=[1, 1, 1, 1])

        # Plot 1: Test-to-test thrust comparison
        ax1 = fig.add_subplot(gs[0, :])
        test_names = list(self.test_results.keys())
        color_list = ['blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive']
        colors = [color_list[i % len(color_list)] for i in range(len(test_names))]

        all_pulse_data = []  # For calculating overall mean

        for i, (test_name, test_data) in enumerate(self.test_results.items()):
            pulse_results = test_data['pulse_results']
            pulse_numbers = list(pulse_results.keys())
            mean_thrusts = [pulse_results[p]['mean_thrust'] for p in pulse_numbers]
            all_pulse_data.extend(mean_thrusts)

            ax1.plot(pulse_numbers, mean_thrusts, 'o-', label=f"{test_name} ({test_data['filename']})",
                    color=colors[i], linewidth=2, markersize=4)

            test_mean = np.mean(mean_thrusts)  # type: ignore[arg-type]
            ax1.axhline(y=test_mean, color=colors[i], linestyle=':', alpha=0.7, linewidth=2,  # type: ignore[arg-type]
                       label=f"{test_name} Mean: {test_mean:.2f}g")

        overall_mean = np.mean(all_pulse_data)  # type: ignore[arg-type]
        ax1.axhline(y=overall_mean, color='red', linestyle='--', linewidth=3, alpha=0.8,  # type: ignore[arg-type]
                   label=f"Combined Mean: {overall_mean:.2f}g")

        ax1.set_xlabel('Pulse Number')
        ax1.set_ylabel('Mean Thrust (g)')
        ax1.set_title(f'Thruster {self.thruster_number}: Test-to-Test Comparison')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)

        # Plot 2: Thrust distribution comparison
        ax2 = fig.add_subplot(gs[1, 0])
        thrust_data_by_test = []
        labels = []

        for test_name, test_data in self.test_results.items():
            pulse_results = test_data['pulse_results']
            mean_thrusts = [pulse_results[p]['mean_thrust'] for p in pulse_results.keys()]
            thrust_data_by_test.append(mean_thrusts)
            labels.append(f"Test {test_name.split('_')[1]}")

        bp = ax2.boxplot(thrust_data_by_test, tick_labels=labels, patch_artist=True)
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        ax2.set_ylabel('Thrust (g)')
        ax2.set_title('Thrust Distribution by Test')
        ax2.grid(True, alpha=0.3)

        ax3 = fig.add_subplot(gs[1, 1])
        metrics = ['overall_mean_thrust', 'overall_peak_thrust']
        metric_labels = ['Mean', 'Peak']

        test_metrics = {metric: [] for metric in metrics}
        for test_data in self.test_results.values():
            for metric in metrics:
                test_metrics[metric].append(test_data['summary_stats'][metric])

        x = np.arange(len(metric_labels))
        width = 0.35

        for i, test_name in enumerate(test_names):
            values = [test_metrics[metric][i] for metric in metrics]
            ax3.bar(x + i * width/len(test_names), values, width/len(test_names),
                   label=f"Test {test_name.split('_')[1]}", color=colors[i], alpha=0.7)

        ax3.set_xlabel('Metrics')
        ax3.set_ylabel('Thrust (g)')
        ax3.set_title('Summary Statistics Comparison')
        ax3.set_xticks(x + width/4)
        ax3.set_xticklabels(metric_labels)
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Plot 4: Repeatability comparison
        ax4 = fig.add_subplot(gs[1, 2])
        repeatability_cvs = [test_data['summary_stats']['thrust_repeatability_cv']
                           for test_data in self.test_results.values()]
        test_labels = [f"Test {name.split('_')[1]}" for name in test_names]

        bars = ax4.bar(test_labels, repeatability_cvs, color=colors, alpha=0.7)
        ax4.set_ylabel('Coefficient of Variation (%)')
        ax4.set_title('Test Repeatability (Lower = Better)')
        ax4.grid(True, alpha=0.3)

        # Add value labels on bars
        for bar, cv in zip(bars, repeatability_cvs):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{cv:.1f}%', ha='center', va='bottom', fontweight='bold')

        ax5 = fig.add_subplot(gs[2, 0])
        ax5.axis('off')

        summary_text = f"""
THRUSTER {self.thruster_number} MULTI-TEST SUMMARY
{'='*50}

Number of Tests: {self.combined_stats['num_tests']}
Total Pulses Analyzed: {self.combined_stats['total_pulses']}

THRUST RECOMMENDATION:
Thrust Value: {self.combined_stats['final_recommended_thrust_g']:.3f} g
             ({self.combined_stats['final_recommended_thrust_N']:.6f} N)

CONSISTENCY ANALYSIS:
Test-to-Test CV: {self.combined_stats['test_to_test_consistency']:.1f}%
Thrust Range: {self.combined_stats['min_recommended_thrust']:.3f} - {self.combined_stats['max_recommended_thrust']:.3f} g
Average Repeatability: {self.combined_stats['average_repeatability_cv']:.1f}% CV

INDIVIDUAL TEST RESULTS:
"""

        for i, (test_name, thrust) in enumerate(zip(self.combined_stats['individual_test_names'],
                                                   self.combined_stats['individual_test_thrusts'])):
            summary_text += f"  {test_name}: {thrust:.3f} g\n"

        summary_text += f"""
        """

        ax5.text(0.05, 0.95, summary_text, transform=ax5.transAxes, fontsize=9,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

        for i, (test_name, test_data) in enumerate(list(self.test_results.items())[:2]):
            if i < 2:  # Only show first 2 tests to fit in layout
                ax = fig.add_subplot(gs[2, i + 1])
                pulse_results = test_data['pulse_results']
                pulse_numbers = list(pulse_results.keys())
                mean_thrusts = [pulse_results[p]['mean_thrust'] for p in pulse_numbers]
                thrust_stds = [pulse_results[p]['thrust_std'] for p in pulse_numbers]

                ax.errorbar(pulse_numbers, mean_thrusts, yerr=thrust_stds,
                           fmt='o-', capsize=3, linewidth=1.5, markersize=4, color=colors[i])
                ax.set_xlabel('Pulse Number')
                ax.set_ylabel('Thrust (g)')
                ax.set_title(f'{test_name} Detail')
                ax.grid(True, alpha=0.3)

        if len(self.test_results) > 2:
            test_name, test_data = list(self.test_results.items())[2]
            ax = fig.add_subplot(gs[3, 0])
            pulse_results = test_data['pulse_results']
            pulse_numbers = list(pulse_results.keys())
            mean_thrusts = [pulse_results[p]['mean_thrust'] for p in pulse_numbers]
            thrust_stds = [pulse_results[p]['thrust_std'] for p in pulse_numbers]

            ax.errorbar(pulse_numbers, mean_thrusts, yerr=thrust_stds,
                       fmt='o-', capsize=3, linewidth=1.5, markersize=4, color=colors[2])
            ax.set_xlabel('Pulse Number')
            ax.set_ylabel('Thrust (g)')
            ax.set_title(f'{test_name} Detail')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save plot
        plot_filename = f"Thruster{self.thruster_number}_MultiTest_Analysis.png"  # type: ignore[operator]
        plot_path = self.output_dir / plot_filename  # type: ignore[operator]
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Multi-test analysis plot saved to: {plot_path}")


    def save_multi_test_results(self) -> None:
        """Save multi-test analysis results"""
        results_filename = f"Thruster{self.thruster_number}_MultiTest_Results.txt"  # type: ignore[operator]
        results_path = self.output_dir / results_filename  # type: ignore[operator]

        with open(results_path, 'w') as f:
            f.write(f"THRUSTER {self.thruster_number} MULTI-TEST ANALYSIS\n")
            f.write("="*60 + "\n\n")
            f.write(f"Analysis date: {datetime.now()}\n")
            f.write(f"Number of tests: {self.combined_stats['num_tests']}\n\n")

            f.write("INDIVIDUAL TEST FILES:\n")
            f.write("-"*30 + "\n")
            for test_name, test_data in self.test_results.items():
                f.write(f"{test_name}: {test_data['filename']}\n")

            f.write(f"\nCOMBINED STATISTICS:\n")
            f.write("-"*30 + "\n")
            for key, value in self.combined_stats.items():
                if isinstance(value, (int, float)):
                    f.write(f"{key}: {value:.6f}\n")
                elif isinstance(value, list):
                    f.write(f"{key}: {value}\n")
                else:
                    f.write(f"{key}: {value}\n")

            f.write(f"\nFINAL RECOMMENDATION:\n")
            f.write("-"*30 + "\n")
            f.write(f"Thrust (Newtons): {self.combined_stats['final_recommended_thrust_N']:.6f}\n")
            f.write(f"Thrust (grams): {self.combined_stats['final_recommended_thrust_g']:.3f}\n")
            f.write(f"Test-to-test consistency: {self.combined_stats['test_to_test_consistency']:.1f}% CV\n")

        print(f"Multi-test results saved to: {results_path}")

    def run_complete_multi_test_analysis(self) -> Dict[str, Any]:
        """Run complete multi-test analysis"""
        print(f"Starting multi-test analysis for Thruster {self.thruster_number}")

        self.analyze_all_tests()
        self.calculate_combined_statistics()
        self.create_multi_test_plots()
        self.save_multi_test_results()

        print(f" Multi-test analysis complete for Thruster {self.thruster_number}")
        return self.combined_stats


class CrossThrusterComparison:
    """Compare performance across all thrusters"""

    def __init__(self, data_dir: str = "Data/Thruster_Data", existing_analyses: Dict = None):  # type: ignore[arg-type]
        """Initialize with data directory and optionally existing analyses"""
        self.data_dir = pathlib.Path(data_dir)
        self.thruster_data = existing_analyses if existing_analyses else {}
        self.comparison_results = {}
        self.output_dir = None
        self._create_output_directory()

    def _create_output_directory(self):
        """Create output directory for cross-thruster comparison"""
        folder_name = "CrossThruster_Comparison"

        self.output_dir = self.data_dir / folder_name
        self.output_dir.mkdir(exist_ok=True)

        print(f" Created cross-thruster comparison directory: {self.output_dir}")

    def find_thruster_files(self) -> Dict[int, List[str]]:
        """Find all test files organized by thruster number"""
        csv_files = list(self.data_dir.glob("*.csv"))
        thruster_files = defaultdict(list)

        for csv_file in csv_files:
            patterns = [
                r'Thruster\((\d+)\)',
                r'thruster_(\d+)',
                r'Thruster(\d+)',
            ]

            for pattern in patterns:
                match = re.search(pattern, csv_file.name, re.IGNORECASE)
                if match:
                    thruster_num = int(match.group(1))
                    thruster_files[thruster_num].append(str(csv_file))
                    break

        for thruster_num in thruster_files:
            thruster_files[thruster_num].sort()

        return dict(thruster_files)

    def analyze_all_thrusters(self) -> Dict:
        """Analyze all thrusters with their multiple tests"""
        # If we already have existing analyses, don't re-run them
        if self.thruster_data:
            print("Using existing thruster analysis data...")
            return self.thruster_data

        print("Starting cross-thruster analysis...")

        thruster_files = self.find_thruster_files()
        print(f"Found files for {len(thruster_files)} thrusters:")

        for thruster_num, files in thruster_files.items():
            print(f"  Thruster {thruster_num}: {len(files)} test files")

        # Analyze each thruster's multiple tests
        for thruster_num, test_files in thruster_files.items():
            print(f"\n{'='*60}")
            print(f"ANALYZING THRUSTER {thruster_num}")
            print(f"{'='*60}")

            try:
                multi_analyzer = MultiTestThrusterAnalyzer(thruster_num, test_files)
                combined_stats = multi_analyzer.run_complete_multi_test_analysis()
                self.thruster_data[thruster_num] = combined_stats
            except Exception as e:
                print(f" Error analyzing Thruster {thruster_num}: {e}")
                continue

        return self.thruster_data

    def calculate_comparison_statistics(self) -> Dict:
        """Calculate comparison statistics across all thrusters"""
        print("\nCalculating cross-thruster comparison statistics...")

        if not self.thruster_data:
            print("No thruster data available")
            return {}

        thruster_numbers = sorted(self.thruster_data.keys())
        thrust_values = [self.thruster_data[t]['final_recommended_thrust_N'] for t in thruster_numbers]
        test_to_test_cvs = [self.thruster_data[t]['test_to_test_consistency'] for t in thruster_numbers]
        avg_repeatabilities = [self.thruster_data[t]['average_repeatability_cv'] for t in thruster_numbers]
        num_tests = [self.thruster_data[t]['num_tests'] for t in thruster_numbers]

        self.comparison_results = {'thruster_numbers': thruster_numbers,
            'thrust_values_N': thrust_values,
            'thrust_values_g': [t * 1000 / 9.81 for t in thrust_values],

            # Fleet-wide statistics
            'fleet_mean_thrust_N': np.mean(thrust_values),
            'fleet_std_thrust_N': np.std(thrust_values),
            'fleet_cv_thrust': np.std(thrust_values) / np.mean(thrust_values) * 100,
            'fleet_min_thrust_N': np.min(thrust_values),
            'fleet_max_thrust_N': np.max(thrust_values),
            'fleet_thrust_range_N': np.max(thrust_values) - np.min(thrust_values),

            # Performance consistency
            'avg_test_to_test_cv': np.mean(test_to_test_cvs),
            'avg_repeatability_cv': np.mean(avg_repeatabilities),
            'best_thruster': thruster_numbers[np.argmin(test_to_test_cvs)],
            'worst_thruster': thruster_numbers[np.argmax(test_to_test_cvs)],
            'most_powerful_thruster': thruster_numbers[np.argmax(thrust_values)],
            'least_powerful_thruster': thruster_numbers[np.argmin(thrust_values)],

            # Quality metrics
            'thrust_uniformity_grade': self._grade_thrust_uniformity(thrust_values),
            'reliability_grade': self._grade_reliability(test_to_test_cvs),

            'individual_test_to_test_cvs': test_to_test_cvs,
            'individual_repeatabilities': avg_repeatabilities,
            'individual_num_tests': num_tests
        }

        return self.comparison_results

    def _grade_thrust_uniformity(self, thrust_values: List[float]) -> str:
        """Grade the uniformity of thrust across the thruster fleet"""
        cv = np.std(thrust_values) / np.mean(thrust_values) * 100

        if cv < 5:
            return "Excellent (< 5% CV)"
        elif cv < 10:
            return "Good (5-10% CV)"
        elif cv < 15:
            return "Fair (10-15% CV)"
        else:
            return "Poor (> 15% CV)"

    def _grade_reliability(self, test_to_test_cvs: List[float]) -> str:
        """Grade the reliability based on test-to-test consistency"""
        avg_cv = np.mean(test_to_test_cvs)

        if avg_cv < 3:
            return "Excellent (< 3% avg CV)"
        elif avg_cv < 5:
            return "Good (3-5% avg CV)"
        elif avg_cv < 8:
            return "Fair (5-8% avg CV)"
        else:
            return "Poor (> 8% avg CV)"

    def create_cross_thruster_plots(self) -> None:
        """Create comprehensive cross-thruster comparison plots"""
        print("\nCreating cross-thruster comparison plots...")

        if len(self.thruster_data) < 2:
            print("Need at least 2 thrusters for comparison")
            return

        # Set up plotting
        plt.style.use('default')
        fig = plt.figure(figsize=(24, 24))  # Increased height for more thruster plots
        gs = fig.add_gridspec(6, 4, height_ratios=[1, 1, 1, 1, 1, 1])  # Added more rows for all thrusters

        thruster_numbers = self.comparison_results['thruster_numbers']
        thrust_values_N = self.comparison_results['thrust_values_N']
        thrust_values_g = self.comparison_results['thrust_values_g']

        ax1 = fig.add_subplot(gs[0, :2])
        bars = ax1.bar(thruster_numbers, thrust_values_g, alpha=0.7,  # type: ignore[attr-defined]
                      color=plt.cm.viridis(np.linspace(0, 1, len(thruster_numbers))))  # type: ignore[attr-defined]
        ax1.axhline(y=self.comparison_results['fleet_mean_thrust_N'] * 1000/9.81, color='red',
                   linestyle='--', linewidth=2, label=f"Fleet Mean: {self.comparison_results['fleet_mean_thrust_N'] * 1000/9.81:.2f}g")
        ax1.set_xlabel('Thruster Number')
        ax1.set_ylabel('Recommended Thrust (g)')
        ax1.set_title('Thrust Comparison Across All Thrusters', fontsize=14, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Add value labels on bars
        for bar, thrust in zip(bars, thrust_values_g):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{thrust:.3f}', ha='center', va='bottom', fontweight='bold')

        # Plot 2: Fleet statistics summary
        ax2 = fig.add_subplot(gs[0, 2:])
        ax2.axis('off')

        fleet_summary = f"""
THRUSTER FLEET PERFORMANCE SUMMARY
{'='*40}

Fleet Size: {len(thruster_numbers)} thrusters
Total Tests: {sum(self.comparison_results['individual_num_tests'])}

THRUST CHARACTERISTICS:
Mean: {self.comparison_results['fleet_mean_thrust_N']:.6f} N ({self.comparison_results['fleet_mean_thrust_N'] * 1000/9.81:.3f} g)
Std: ±{self.comparison_results['fleet_std_thrust_N']:.6f} N (±{self.comparison_results['fleet_std_thrust_N'] * 1000/9.81:.3f} g)
Range: {self.comparison_results['fleet_min_thrust_N']:.6f} - {self.comparison_results['fleet_max_thrust_N']:.6f} N
Fleet CV: {self.comparison_results['fleet_cv_thrust']:.1f}%
        """

        ax2.text(0.05, 0.95, fleet_summary, transform=ax2.transAxes, fontsize=10,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

        # Plot 3: Test-to-test consistency comparison
        ax3 = fig.add_subplot(gs[1, 0])
        test_cvs = self.comparison_results['individual_test_to_test_cvs']
        bars = ax3.bar(thruster_numbers, test_cvs, alpha=0.7, color='orange')
        ax3.set_xlabel('Thruster Number')
        ax3.set_ylabel('Test-to-Test CV (%)')
        ax3.set_title('Test-to-Test Consistency\n(Lower = Better)')
        ax3.grid(True, alpha=0.3)

        ax4 = fig.add_subplot(gs[1, 1])
        ax4.hist(thrust_values_g, bins=min(10, len(thrust_values_g)), alpha=0.7, edgecolor='black', color='skyblue')  # type: ignore[arg-type]
        ax4.axvline(np.mean(thrust_values_g), color='red', linestyle='--', linewidth=2,  # type: ignore[arg-type]
                   label=f'Mean: {np.mean(thrust_values_g):.3f}g')
        ax4.set_xlabel('Thrust (g)')
        ax4.set_ylabel('Number of Thrusters')
        ax4.set_title('Fleet Thrust Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        ax5 = fig.add_subplot(gs[1, 2:])
        ax5.scatter(thrust_values_g, test_cvs, s=100, alpha=0.7, c=thruster_numbers, cmap='viridis')
        for i, thruster_num in enumerate(thruster_numbers):
            ax5.annotate(f'T{thruster_num}', (thrust_values_g[i], test_cvs[i]),
                        xytext=(5, 5), textcoords='offset points', fontsize=8)
        ax5.set_xlabel('Thrust (g)')
        ax5.set_ylabel('Test-to-Test CV (%)')
        ax5.set_title('Thrust vs Consistency')
        ax5.grid(True, alpha=0.3)

        for idx, thruster_num in enumerate(thruster_numbers):
            row = 2 + idx // 4  # 2 rows of 4 plots each
            col = idx % 4
            ax = fig.add_subplot(gs[row, col])

            thruster_stats = self.thruster_data[thruster_num]
            individual_thrusts = thruster_stats['individual_test_thrusts']
            test_names = [name.split('_')[1] for name in thruster_stats['individual_test_names']]
  # type: ignore[attr-defined]
            ax.bar(test_names, individual_thrusts, alpha=0.7, color=plt.cm.viridis(idx/len(thruster_numbers)))  # type: ignore[attr-defined]
            ax.axhline(y=thruster_stats['final_recommended_thrust_g'], color='red',
                      linestyle='--', label=f"Final: {thruster_stats['final_recommended_thrust_g']:.3f}g")
            ax.set_xlabel('Test Number')
            ax.set_ylabel('Thrust (g)')
            ax.set_title(f'Thruster {thruster_num} Test Results')
            ax.legend()
            ax.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save plot
        plot_filename = "Cross_Thruster_Comparison.png"  # type: ignore[operator]
        plot_path = self.output_dir / plot_filename  # type: ignore[operator]
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Cross-thruster comparison plot saved to: {plot_path}")

    def generate_mpc_configuration_file(self) -> None:
        """Generate SET_EFFECTS matrix configuration for MPC"""
        print("\nGenerating MPC configuration file...")

        config_filename = "MPC_SET_EFFECTS_Configuration.txt"  # type: ignore[operator]
        config_path = self.output_dir / config_filename  # type: ignore[operator]

        with open(config_path, 'w') as f:
            f.write("MPC THRUSTER SET_EFFECTS MATRIX CONFIGURATION\n")
            f.write("="*60 + "\n\n")
            f.write(f"Generated: {datetime.now()}\n")
            f.write(f"Based on analysis of {len(self.thruster_data)} thrusters\n\n")

            f.write("RECOMMENDED THRUST VALUES (Newtons):\n")
            f.write("-"*40 + "\n")

            for thruster_num in sorted(self.thruster_data.keys()):
                thrust_N = self.thruster_data[thruster_num]['final_recommended_thrust_N']
                thrust_g = self.thruster_data[thruster_num]['final_recommended_thrust_g']
                cv = self.thruster_data[thruster_num]['test_to_test_consistency']
                f.write(f"Thruster {thruster_num}: {thrust_N:.6f} N ({thrust_g:.3f} g, CV: {cv:.1f}%)\n")

            f.write(f"\nFLEET STATISTICS:\n")
            f.write("-"*20 + "\n")
            f.write(f"Mean thrust: {self.comparison_results['fleet_mean_thrust_N']:.6f} N\n")
            f.write(f"Fleet CV: {self.comparison_results['fleet_cv_thrust']:.1f}%\n")
            f.write(f"Thrust uniformity: {self.comparison_results['thrust_uniformity_grade']}\n")
            f.write(f"Reliability grade: {self.comparison_results['reliability_grade']}\n")

            f.write(f"\nPYTHON CODE FOR SET_EFFECTS MATRIX:\n")
            f.write("-"*35 + "\n")
            f.write("# Paste this into your MPC configuration\n")
            f.write("SET_EFFECTS = [\n")

            for thruster_num in sorted(self.thruster_data.keys()):
                thrust_N = self.thruster_data[thruster_num]['final_recommended_thrust_N']
                f.write(f"    {thrust_N:.6f},  # Thruster {thruster_num}\n")

            f.write("]\n")

            f.write(f"\nQUALITY NOTES:\n")
            f.write("-"*15 + "\n")
            f.write(f"- Best performing thruster: {self.comparison_results['best_thruster']}\n")
            f.write(f"- Worst performing thruster: {self.comparison_results['worst_thruster']}\n")
            f.write(f"- Most powerful thruster: {self.comparison_results['most_powerful_thruster']}\n")
            f.write(f"- Least powerful thruster: {self.comparison_results['least_powerful_thruster']}\n")
            f.write(f"- Fleet thrust range: {self.comparison_results['fleet_thrust_range_N']:.6f} N\n")

        print(f"MPC configuration saved to: {config_path}")

    def save_comparison_results(self) -> None:
        """Save detailed comparison results"""
        results_filename = "Cross_Thruster_Analysis_Results.txt"  # type: ignore[operator]
        results_path = self.output_dir / results_filename  # type: ignore[operator]

        with open(results_path, 'w') as f:
            f.write("CROSS-THRUSTER ANALYSIS RESULTS\n")
            f.write("="*50 + "\n\n")
            f.write(f"Analysis date: {datetime.now()}\n")
            f.write(f"Number of thrusters: {len(self.thruster_data)}\n\n")

            f.write("COMPARISON STATISTICS:\n")
            f.write("-"*30 + "\n")
            for key, value in self.comparison_results.items():
                if isinstance(value, (int, float)):
                    f.write(f"{key}: {value:.6f}\n")
                elif isinstance(value, list) and len(value) < 20:
                    f.write(f"{key}: {value}\n")
                else:
                    f.write(f"{key}: {value}\n")

            f.write(f"\nINDIVIDUAL THRUSTER SUMMARIES:\n")
            f.write("-"*35 + "\n")
            for thruster_num, data in self.thruster_data.items():
                f.write(f"\nThruster {thruster_num}:\n")
                f.write(f"  Final recommendation: {data['final_recommended_thrust_N']:.6f} N\n")
                f.write(f"  Test-to-test consistency: {data['test_to_test_consistency']:.1f}% CV\n")
                f.write(f"  Number of tests: {data['num_tests']}\n")
                f.write(f"  Average repeatability: {data['average_repeatability_cv']:.1f}% CV\n")

        print(f"Detailed comparison results saved to: {results_path}")

    def run_complete_comparison(self) -> Dict:
        """Run complete cross-thruster comparison analysis"""
        print("Starting complete cross-thruster comparison analysis...")

        self.analyze_all_thrusters()
        self.calculate_comparison_statistics()
        self.create_cross_thruster_plots()
        self.generate_mpc_configuration_file()
        self.save_comparison_results()

        print(" Complete cross-thruster analysis finished!")
        return self.comparison_results


def analyze_all_thruster_files(data_dir: str = "Data/Thruster_Data") -> None:
    """Analyze all CSV files in the thruster data directory (legacy single-file analysis)"""
    data_path = pathlib.Path(data_dir)
    csv_files = list(data_path.glob("*.csv"))

    if not csv_files:
        print(f"No CSV files found in {data_path}")
        return

    print(f"Found {len(csv_files)} CSV files to analyze:")
    for i, file in enumerate(csv_files, 1):
        print(f"  {i}. {file.name}")

    # Analyze each file
    for csv_file in csv_files:
        print(f"\n{'='*80}")
        print(f"ANALYZING: {csv_file.name}")
        print(f"{'='*80}")

        try:
            analyzer = ThrusterDataAnalyzer(str(csv_file))
            analyzer.run_complete_analysis()
        except Exception as e:
            print(f" Error analyzing {csv_file.name}: {e}")
            continue


def analyze_multi_test_thrusters(data_dir: str = "Data/Thruster_Data") -> None:
    """Analyze all thrusters with their multiple tests"""
    print("Starting multi-test thruster analysis...")

    # Find thruster files
    data_path = pathlib.Path(data_dir)
    csv_files = list(data_path.glob("*.csv"))
    thruster_files = defaultdict(list)

    for csv_file in csv_files:
        patterns = [
            r'Thruster\((\d+)\)',
            r'thruster_(\d+)',
            r'Thruster(\d+)',
        ]

        for pattern in patterns:
            match = re.search(pattern, csv_file.name, re.IGNORECASE)
            if match:
                thruster_num = int(match.group(1))
                thruster_files[thruster_num].append(str(csv_file))
                break

    for thruster_num in thruster_files:
        thruster_files[thruster_num].sort()

    print(f"Found {len(thruster_files)} thrusters with multiple tests:")
    for thruster_num, files in sorted(thruster_files.items()):
        print(f"  Thruster {thruster_num}: {len(files)} test files")

    # Analyze each thruster's multiple tests
    for thruster_num, test_files in sorted(thruster_files.items()):
        print(f"\n{'='*80}")
        print(f"MULTI-TEST ANALYSIS: THRUSTER {thruster_num}")
        print(f"{'='*80}")

        try:
            multi_analyzer = MultiTestThrusterAnalyzer(thruster_num, test_files)
            multi_analyzer.run_complete_multi_test_analysis()
        except Exception as e:
            print(f" Error analyzing Thruster {thruster_num}: {e}")
            continue


def run_cross_thruster_comparison(data_dir: str = "Data/Thruster_Data") -> None:
    """Run comprehensive cross-thruster comparison analysis"""
    print("Starting cross-thruster comparison analysis...")

    try:
        cross_analyzer = CrossThrusterComparison(data_dir)
        cross_analyzer.run_complete_comparison()
    except Exception as e:
        print(f" Error in cross-thruster analysis: {e}")


def run_multi_test_analysis_and_collect_results(data_dir: str = "Data/Thruster_Data") -> Dict:
    """Run multi-test analysis and return collected results for cross-thruster comparison"""
    print("Starting multi-test thruster analysis...")

    # Find thruster files
    data_path = pathlib.Path(data_dir)
    csv_files = list(data_path.glob("*.csv"))
    thruster_files = defaultdict(list)

    for csv_file in csv_files:
        patterns = [
            r'Thruster\((\d+)\)',
            r'thruster_(\d+)',
            r'Thruster(\d+)',
        ]

        for pattern in patterns:
            match = re.search(pattern, csv_file.name, re.IGNORECASE)
            if match:
                thruster_num = int(match.group(1))
                thruster_files[thruster_num].append(str(csv_file))
                break

    for thruster_num in thruster_files:
        thruster_files[thruster_num].sort()

    print(f"Found {len(thruster_files)} thrusters with multiple tests:")
    for thruster_num, files in sorted(thruster_files.items()):
        print(f"  Thruster {thruster_num}: {len(files)} test files")

    # Collect analysis results
    thruster_analyses = {}

    # Analyze each thruster's multiple tests and collect results
    for thruster_num, test_files in sorted(thruster_files.items()):
        print(f"\n{'='*80}")
        print(f"MULTI-TEST ANALYSIS: THRUSTER {thruster_num}")
        print(f"{'='*80}")

        try:
            multi_analyzer = MultiTestThrusterAnalyzer(thruster_num, test_files)
            combined_stats = multi_analyzer.run_complete_multi_test_analysis()
            thruster_analyses[thruster_num] = combined_stats
        except Exception as e:
            print(f" Error analyzing Thruster {thruster_num}: {e}")
            continue

    return thruster_analyses


def main():
    """Main function for command-line usage"""
    import sys

    if len(sys.argv) > 1:
        command = sys.argv[1].lower()

        if command == "single":
            # Analyze specific file
            if len(sys.argv) > 2:
                csv_file = sys.argv[2]
                if pathlib.Path(csv_file).exists():
                    analyzer = ThrusterDataAnalyzer(csv_file)
                    analyzer.run_complete_analysis()
                else:
                    print(f"File not found: {csv_file}")
            else:
                print("Usage: python script.py single <csv_file_path>")

        elif command == "multi":
            # Analyze all thrusters with multiple tests
            data_dir = sys.argv[2] if len(sys.argv) > 2 else "Data/Thruster_Data"
            analyze_multi_test_thrusters(data_dir)

        elif command == "compare":
            # Run cross-thruster comparison
            data_dir = sys.argv[2] if len(sys.argv) > 2 else "Data/Thruster_Data"
            run_cross_thruster_comparison(data_dir)

        elif command == "all":
            # Run complete analysis pipeline without duplicates
            data_dir = sys.argv[2] if len(sys.argv) > 2 else "Data/Thruster_Data"
            print("Running complete analysis pipeline...")
            print("Step 1: Multi-test analysis for each thruster")

            # First run multi-test analysis and collect results
            thruster_analyses = run_multi_test_analysis_and_collect_results(data_dir)

            print("\nStep 2: Cross-thruster comparison using existing results")
            cross_analyzer = CrossThrusterComparison(data_dir, existing_analyses=thruster_analyses)
            cross_analyzer.calculate_comparison_statistics()
            cross_analyzer.create_cross_thruster_plots()
            cross_analyzer.generate_mpc_configuration_file()
            cross_analyzer.save_comparison_results()
            print(" Complete cross-thruster analysis finished!")

        else:
            print("Unknown command. Available commands:")
            print("  single <file>  - Analyze a single CSV file")
            print("  multi [dir]    - Analyze multiple tests per thruster")
            print("  compare [dir]  - Run cross-thruster comparison")
            print("  all [dir]      - Run complete analysis pipeline")

    else:
        # Default: run complete analysis pipeline without duplicates
        print("Running complete analysis pipeline (default)...")
        print("Use 'python script.py help' for command options")
        print("\nStep 1: Multi-test analysis for each thruster")

        # First run multi-test analysis and collect results
        thruster_analyses = run_multi_test_analysis_and_collect_results()

        print("\nStep 2: Cross-thruster comparison using existing results")
        cross_analyzer = CrossThrusterComparison("Data/Thruster_Data", existing_analyses=thruster_analyses)
        cross_analyzer.calculate_comparison_statistics()
        cross_analyzer.create_cross_thruster_plots()
        cross_analyzer.generate_mpc_configuration_file()
        cross_analyzer.save_comparison_results()
        print(" Complete cross-thruster analysis finished!")


if __name__ == "__main__":
    main()
