using System;
using System.Collections.Generic;
using System.Windows.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using TubeLaserCAM.UI.Helpers;
using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace TubeLaserCAM.Models
{
    public class AnimationController3D : ObservableObject
    {
        private DispatcherTimer _timer;
        private List<GCodeCommand3D> _commands;
        private int _currentCommandIndex;
        private TimeSpan _elapsedTimeInCurrentSegment;
        private TimeSpan _estimatedSegmentDuration;
        private TimeSpan _totalEstimatedSimulationTime;
        private double _tubeRadius = 25;
        private int _tickCount = 0;
        private double _lastDebuggedC = 0;
        private DateTime _lastDebugTime = DateTime.MinValue;
        public List<GCodeCommand3D> Commands => _commands;
        public int CurrentCommandIndex => _currentCommandIndex;



        public AnimationController3D()
        {
            _timer = new DispatcherTimer();
            _timer.Interval = TimeSpan.FromMilliseconds(16);
            _timer.Tick += OnTimerTick;
            _commands = new List<GCodeCommand3D>();
        }

        private double _speedRatio = 1.0;
        public double SpeedRatio
        {
            get => _speedRatio;
            set => SetProperty(ref _speedRatio, Math.Max(0.1, value));
        }

        private bool _isPlaying;
        public bool IsPlaying
        {
            get => _isPlaying;
            private set => SetProperty(ref _isPlaying, value);
        }

        private double _currentYPosition;
        public double CurrentYPosition
        {
            get => _currentYPosition;
            private set => SetProperty(ref _currentYPosition, value);
        }

        private double _currentCRotation;
        public double CurrentCRotation
        {
            get => _currentCRotation;
            private set
            {
                if (Math.Abs(_currentCRotation - value) > 0.001)
                {
                    SetProperty(ref _currentCRotation, value);
                    System.Diagnostics.Debug.WriteLine($"[ANIM] CurrentCRotation changed to: {value:F3}°");
                }
            }
        }

        private double _currentZPosition;
        public double CurrentZPosition
        {
            get => _currentZPosition;
            private set => SetProperty(ref _currentZPosition, value);
        }

        private bool _isCurrentLaserOn;
        public bool IsCurrentLaserOn
        {
            get => _isCurrentLaserOn;
            private set => SetProperty(ref _isCurrentLaserOn, value);
        }

        private double _simulationProgress;
        public double SimulationProgress
        {
            get => _simulationProgress;
            private set => SetProperty(ref _simulationProgress, value);
        }

        public void LoadCommands(List<GCodeCommand3D> commands, double tubeRadius)
        {
            _commands = commands ?? new List<GCodeCommand3D>();
            _tubeRadius = tubeRadius > 0 ? tubeRadius : 0.001;

            _totalEstimatedSimulationTime = TimeSpan.Zero;
            if (_commands != null && _commands.Count > 0)
            {
                GCodeCommand3D prevCmd = GetPreviousStateCommand(0);
                for (int i = 0; i < _commands.Count; i++)
                {
                    _totalEstimatedSimulationTime += CalculateSingleSegmentDuration(prevCmd, _commands[i]);
                    prevCmd = _commands[i];
                }
            }
            Reset();
        }


        public void Play()
        {
            if (_commands.Count == 0) return;

            if (_currentCommandIndex >= _commands.Count)
            {
                Reset();
            }

            if (_currentCommandIndex < _commands.Count)
            {
                IsPlaying = true;
                if (_elapsedTimeInCurrentSegment == TimeSpan.Zero)
                {
                    SetCurrentSegment(_currentCommandIndex);
                }
                _timer.Start();
            }
        }

        public void Pause()
        {
            IsPlaying = false;
            _timer.Stop();
        }

        public void Stop()
        {
            Pause();
            Reset();
        }

        public void Reset()
        {
            Pause();
            _currentCommandIndex = 0;
            _elapsedTimeInCurrentSegment = TimeSpan.Zero;
            SimulationProgress = 0;

            if (_commands != null && _commands.Count > 0)
            {
                var firstCmd = _commands[0];
                CurrentYPosition = firstCmd.Y;
                CurrentCRotation = firstCmd.C;
                CurrentZPosition = firstCmd.Z;
                IsCurrentLaserOn = firstCmd.IsLaserOn;
                SetCurrentSegment(_currentCommandIndex);
            }
            else
            {
                CurrentYPosition = 0;
                CurrentCRotation = 0;
                CurrentZPosition = 20;
                IsCurrentLaserOn = false;
                _estimatedSegmentDuration = TimeSpan.Zero;
            }

            OnPropertyChanged(nameof(CurrentYPosition));
            OnPropertyChanged(nameof(CurrentCRotation));
            OnPropertyChanged(nameof(CurrentZPosition));
            OnPropertyChanged(nameof(IsCurrentLaserOn));
            OnPropertyChanged(nameof(SimulationProgress));
            OnPropertyChanged(nameof(IsPlaying));
        }

        private GCodeCommand3D GetPreviousStateCommand(int currentIndex)
        {
            if (currentIndex == 0 || _commands == null || _commands.Count == 0)
            {
                double initialZ = 20;
                if (_commands != null && _commands.Count > 0 && _commands[0] != null)
                {
                    initialZ = _commands[0].Z;
                }
                return new GCodeCommand3D(0, 0, initialZ, 0, false, true, GCodeCommandType.Other, "PSEUDO_START_FOR_CALC");
            }
            return _commands[currentIndex - 1];
        }

        private void SetCurrentSegment(int commandIndex)
        {
            if (commandIndex < 0 || commandIndex >= _commands.Count)
            {
                Pause();
                return;
            }
            _elapsedTimeInCurrentSegment = TimeSpan.Zero;
            _estimatedSegmentDuration = CalculateSingleSegmentDuration(GetPreviousStateCommand(commandIndex), _commands[commandIndex]);
        }

        private void OnTimerTick(object sender, EventArgs e)
        {
            if (!_isPlaying || _commands.Count == 0 || _currentCommandIndex >= _commands.Count)
            {
                Pause();
                return;
            }

            _tickCount++;
            _elapsedTimeInCurrentSegment += TimeSpan.FromMilliseconds(_timer.Interval.TotalMilliseconds * SpeedRatio);

            GCodeCommand3D targetCmd = _commands[_currentCommandIndex];
            GCodeCommand3D startCmd = GetPreviousStateCommand(_currentCommandIndex);

            double segmentProgress = 0;
            if (_estimatedSegmentDuration.TotalSeconds > 0.00001)
            {
                segmentProgress = Math.Min(1.0, _elapsedTimeInCurrentSegment.TotalSeconds / _estimatedSegmentDuration.TotalSeconds);
            }
            else
            {
                segmentProgress = 1.0;
            }

            // Store old values for comparison
            double oldY = CurrentYPosition;
            double oldC = CurrentCRotation;
            double oldZ = CurrentZPosition;
            bool oldLaserOn = IsCurrentLaserOn;

            // Update positions - properties will fire PropertyChanged if value changes
            CurrentYPosition = startCmd.Y + (targetCmd.Y - startCmd.Y) * segmentProgress;
            CurrentZPosition = startCmd.Z + (targetCmd.Z - startCmd.Z) * segmentProgress;

            // QUAN TRỌNG: Sử dụng angle interpolation với xử lý vùng giao
            CurrentCRotation = AngleHelper.InterpolateAngle(startCmd.C, targetCmd.C, segmentProgress);

            IsCurrentLaserOn = targetCmd.IsLaserOn;

            // GIẢM DEBUG: Chỉ log mỗi 100 ticks (thay vì 10) VÀ khi có thay đổi lớn
            bool shouldLogDebug = false;

            // Log định kỳ mỗi 100 ticks (khoảng 1.6 giây ở 60fps)
            if (_tickCount % 100 == 0)
            {
                shouldLogDebug = true;
            }
            // Hoặc khi có thay đổi góc C lớn (> 10 độ)
            else if (Math.Abs(CurrentCRotation - _lastDebuggedC) > 10.0)
            {
                shouldLogDebug = true;
            }
            // Hoặc khi thay đổi command index (segment mới)
            else if (_currentCommandIndex != _lastLoggedIndex)
            {
                shouldLogDebug = true;
            }

            if (shouldLogDebug)
            {
                double deltaC = CurrentCRotation - _lastDebuggedC;
                System.Diagnostics.Debug.WriteLine(
                    $"[ANIM] Tick {_tickCount} | Cmd {_currentCommandIndex}: " +
                    $"Y={CurrentYPosition:F3}, C={CurrentCRotation:F3}°, " +
                    $"Progress={segmentProgress:P}"
                );
                _lastDebuggedC = CurrentCRotation;
                _lastDebugTime = DateTime.Now;
            }

            // GIẢM DEBUG: Chỉ log seam crossing quan trọng
            if (_currentCommandIndex != _lastLoggedIndex && AngleHelper.CrossesSeam(startCmd.C, targetCmd.C))
            {
                _lastLoggedIndex = _currentCommandIndex;
                double delta = AngleHelper.ShortestAngleDelta(startCmd.C, targetCmd.C);

                // Chỉ log nếu góc xoay > 90 độ (quan trọng)
                if (Math.Abs(delta) > 90)
                {
                    System.Diagnostics.Debug.WriteLine(
                        $"[SEAM CROSSING] Cmd {_currentCommandIndex}: " +
                        $"C rotation {startCmd.C:F1}° → {targetCmd.C:F1}° " +
                        $"Delta: {delta:F1}°"
                    );
                }
            }

            // Khi hoàn thành segment
            if (segmentProgress >= 1.0)
            {
                // Force set to exact target values
                CurrentYPosition = targetCmd.Y;
                CurrentCRotation = targetCmd.C;
                CurrentZPosition = targetCmd.Z;
                IsCurrentLaserOn = targetCmd.IsLaserOn;

                // GIẢM DEBUG: Chỉ log completion cho các milestone (mỗi 10 segments)
                if (_currentCommandIndex % 10 == 0 || _currentCommandIndex == _commands.Count - 1)
                {
                    System.Diagnostics.Debug.WriteLine(
                        $"[COMPLETE] Segment {_currentCommandIndex}/{_commands.Count} completed. " +
                        $"Y={CurrentYPosition:F3}, C={CurrentCRotation:F3}"
                    );
                }

                _currentCommandIndex++;
                if (_currentCommandIndex < _commands.Count)
                {
                    SetCurrentSegment(_currentCommandIndex);
                }
                else
                {
                    Pause();
                    SimulationProgress = 1.0;

                    // Log khi hoàn thành toàn bộ simulation
                    System.Diagnostics.Debug.WriteLine("[SIMULATION] Completed all segments");
                    return;
                }
            }

            // Update overall progress
            UpdateSimulationProgress();
        }




        private int _lastLoggedIndex = -1; 
        private TimeSpan CalculateSingleSegmentDuration(GCodeCommand3D startCmd, GCodeCommand3D targetCmd)
        {
            double deltaY = targetCmd.Y - startCmd.Y;
            double deltaZ = targetCmd.Z - startCmd.Z;

            // QUAN TRỌNG: Sử dụng shortest angle delta
            double deltaC_deg = AngleHelper.ShortestAngleDelta(startCmd.C, targetCmd.C);

            double feedRate = targetCmd.FeedRate;
            if (feedRate <= 0 && !targetCmd.IsRapidMove) feedRate = 3000;
            if (targetCmd.IsRapidMove) feedRate = Math.Max(feedRate, 10000);

            // Tính arc length với absolute value của delta
            double arcLengthC = Math.Abs(deltaC_deg * Math.PI / 180.0 * _tubeRadius);

            // Tổng khoảng cách 3D
            double totalSurfaceDistance = Math.Sqrt(
                deltaY * deltaY +
                arcLengthC * arcLengthC +
                deltaZ * deltaZ
            );

            if (feedRate > 0 && totalSurfaceDistance > 0.0001)
            {
                double timeInSeconds = totalSurfaceDistance / (feedRate / 60.0);

                // Debug log cho các move lớn hoặc vượt seam
                if (AngleHelper.CrossesSeam(startCmd.C, targetCmd.C) || totalSurfaceDistance > 50)
                {
                    System.Diagnostics.Debug.WriteLine(
                        $"Segment duration: Y:{deltaY:F2}mm, C:{deltaC_deg:F2}°, Z:{deltaZ:F2}mm " +
                        $"→ Distance:{totalSurfaceDistance:F2}mm, Time:{timeInSeconds:F3}s"
                    );
                }

                return TimeSpan.FromSeconds(timeInSeconds);
            }

            // M-codes và các lệnh khác
            if (targetCmd.CommandType >= GCodeCommandType.M03 && targetCmd.CommandType <= GCodeCommandType.M05 ||
                targetCmd.CommandType == GCodeCommandType.Other)
            {
                if (totalSurfaceDistance <= 0.0001)
                    return TimeSpan.FromMilliseconds(50);
            }

            return TimeSpan.Zero;
        }
        private void UpdateSimulationProgress()
        {
            if (_totalEstimatedSimulationTime.TotalSeconds > 0)
            {
                TimeSpan currentOverallElapsedTime = TimeSpan.Zero;
                GCodeCommand3D prevCmdForProgress = GetPreviousStateCommand(0);

                for (int i = 0; i < _currentCommandIndex; i++)
                {
                    currentOverallElapsedTime += CalculateSingleSegmentDuration(prevCmdForProgress, _commands[i]);
                    prevCmdForProgress = _commands[i];
                }

                currentOverallElapsedTime += _elapsedTimeInCurrentSegment;
                SimulationProgress = Math.Min(1.0, currentOverallElapsedTime.TotalSeconds / _totalEstimatedSimulationTime.TotalSeconds);
            }
            else
            {
                SimulationProgress = (_commands.Count > 0 && _currentCommandIndex >= _commands.Count) ? 1.0 : 0.0;
            }
        }
        public void DebugCurrentState()
        {
            System.Diagnostics.Debug.WriteLine($"\n=== ANIMATION STATE DEBUG ===");
            System.Diagnostics.Debug.WriteLine($"IsPlaying: {IsPlaying}");
            System.Diagnostics.Debug.WriteLine($"Current Command Index: {_currentCommandIndex}/{_commands?.Count ?? 0}");
            System.Diagnostics.Debug.WriteLine($"Current Position: Y={CurrentYPosition:F3}, C={CurrentCRotation:F3}, Z={CurrentZPosition:F3}");
            System.Diagnostics.Debug.WriteLine($"Laser On: {IsCurrentLaserOn}");
            System.Diagnostics.Debug.WriteLine($"Speed Ratio: {SpeedRatio}");
            System.Diagnostics.Debug.WriteLine($"Simulation Progress: {SimulationProgress:P}");

            if (_currentCommandIndex >= 0 && _currentCommandIndex < _commands?.Count)
            {
                var currentCmd = _commands[_currentCommandIndex];
                var prevCmd = GetPreviousStateCommand(_currentCommandIndex);
                System.Diagnostics.Debug.WriteLine($"\nCurrent Segment:");
                System.Diagnostics.Debug.WriteLine($"  From: Y={prevCmd.Y:F3}, C={prevCmd.C:F3}");
                System.Diagnostics.Debug.WriteLine($"  To: Y={currentCmd.Y:F3}, C={currentCmd.C:F3}");
                System.Diagnostics.Debug.WriteLine($"  Duration: {_estimatedSegmentDuration.TotalSeconds:F3}s");
                System.Diagnostics.Debug.WriteLine($"  Elapsed: {_elapsedTimeInCurrentSegment.TotalSeconds:F3}s");
            }
            System.Diagnostics.Debug.WriteLine("=== END DEBUG ===\n");
        }





    }
}