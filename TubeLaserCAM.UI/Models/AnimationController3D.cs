using System;
using System.Collections.Generic;
using System.Windows.Threading;
using CommunityToolkit.Mvvm.ComponentModel;

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
            private set => SetProperty(ref _currentCRotation, value);
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

        private TimeSpan CalculateSingleSegmentDuration(GCodeCommand3D startCmd, GCodeCommand3D targetCmd)
        {
            double deltaY = targetCmd.Y - startCmd.Y;
            double deltaC_deg = targetCmd.C - startCmd.C;
            double deltaZ = targetCmd.Z - startCmd.Z;

            double feedRate = targetCmd.FeedRate;
            if (feedRate <= 0 && !targetCmd.IsRapidMove) feedRate = 3000;
            if (targetCmd.IsRapidMove) feedRate = Math.Max(feedRate, 10000);

            double arcLengthC = Math.Abs(deltaC_deg * Math.PI / 180.0 * _tubeRadius);
            double totalSurfaceDistance = Math.Sqrt(deltaY * deltaY + arcLengthC * arcLengthC + deltaZ * deltaZ);

            if (feedRate > 0 && totalSurfaceDistance > 0.0001)
            {
                return TimeSpan.FromSeconds(totalSurfaceDistance / (feedRate / 60.0));
            }

            if (targetCmd.CommandType >= GCodeCommandType.M03 && targetCmd.CommandType <= GCodeCommandType.M05 || targetCmd.CommandType == GCodeCommandType.Other)
            {
                if (totalSurfaceDistance <= 0.0001)
                    return TimeSpan.FromMilliseconds(50);
            }
            return TimeSpan.Zero;
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

            CurrentYPosition = startCmd.Y + (targetCmd.Y - startCmd.Y) * segmentProgress;
            CurrentCRotation = startCmd.C + (targetCmd.C - startCmd.C) * segmentProgress;
            CurrentZPosition = startCmd.Z + (targetCmd.Z - startCmd.Z) * segmentProgress;
            IsCurrentLaserOn = targetCmd.IsLaserOn;

            if (segmentProgress >= 1.0)
            {
                CurrentYPosition = targetCmd.Y;
                CurrentCRotation = targetCmd.C;
                CurrentZPosition = targetCmd.Z;
                IsCurrentLaserOn = targetCmd.IsLaserOn;

                _currentCommandIndex++;
                if (_currentCommandIndex < _commands.Count)
                {
                    SetCurrentSegment(_currentCommandIndex);
                }
                else
                {
                    Pause();
                    SimulationProgress = 1.0;
                    OnPropertyChanged(nameof(SimulationProgress));
                    return;
                }
            }

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
                if (_commands.Count == 0) SimulationProgress = 0.0;
            }
        }
    }
}