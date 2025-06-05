using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;

namespace TubeLaserCAM.Models
{
    public class GCodeParser3D
    {
        private double _currentY = 0;
        private double _currentC = 0;
        private double _currentZ = 0;
        private double _currentFeedRate = 0;
        private bool _isLaserOn = false;
        private double _currentLaserPower = 0;
        private GCodeCommandType _modalGCommand = GCodeCommandType.G01;

        public List<GCodeCommand3D> Commands { get; private set; }

        public GCodeParser3D()
        {
            Commands = new List<GCodeCommand3D>();
        }

        public List<GCodeCommand3D> Parse(string filePath)
        {
            Commands.Clear();
            ResetState();

            try
            {
                var lines = File.ReadAllLines(filePath);
                ParseLines(lines);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Error parsing G-code file: {ex.Message}");
                throw;
            }

            return Commands;
        }

        public List<GCodeCommand3D> Parse(IEnumerable<string> gcodeLines)
        {
            Commands.Clear();
            ResetState();

            ParseLines(gcodeLines.ToArray());
            return Commands;
        }

        private void ResetState()
        {
            _currentY = 0;
            _currentC = 0;
            _currentZ = 10; // Start at safe height matching G-code
            _currentFeedRate = 1000; // Default from G-code
            _isLaserOn = false;
            _currentLaserPower = 0;
            _modalGCommand = GCodeCommandType.G01;
        }

        private void ParseLines(string[] lines)
        {
            foreach (var rawLine in lines)
            {
                try
                {
                    ParseSingleLine(rawLine);
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"Error parsing line '{rawLine}': {ex.Message}");
                    // Continue with next line instead of crashing
                }
            }
        }

        private void ParseSingleLine(string rawLine)
        {
            var line = rawLine.Trim().ToUpper();

            // Skip empty lines and comments
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith(";") || line.StartsWith("("))
                return;

            // Remove inline comments
            int commentIndex = line.IndexOf(';');
            if (commentIndex >= 0)
                line = line.Substring(0, commentIndex).Trim();

            // Remove parenthetical comments
            line = Regex.Replace(line, @"\([^)]*\)", "").Trim();

            if (string.IsNullOrWhiteSpace(line))
                return;

            // Parse command parts
            var parts = Regex.Split(line, @"(?=[GMXYZFCIJKSP])").Where(s => !string.IsNullOrWhiteSpace(s)).ToArray();

            // Track what we found in this line
            double y = _currentY;
            double c = _currentC;
            double z = _currentZ;
            double f = _currentFeedRate;
            double laserPower = _currentLaserPower; // Renamed from 's' to avoid conflict
            bool hasMovement = false;
            bool hasGCommand = false;
            bool hasMCommand = false;
            GCodeCommandType currentLineGCommand = _modalGCommand;
            GCodeCommandType? mCommand = null;

            foreach (var part in parts)
            {
                if (part.Length < 1) continue;

                char commandChar = part[0];
                string valueStr = part.Length > 1 ? part.Substring(1) : "";

                switch (commandChar)
                {
                    case 'G':
                        if (TryParseDouble(valueStr, out double gValue))
                        {
                            hasGCommand = true;
                            currentLineGCommand = ParseGCommand((int)gValue);
                        }
                        break;

                    case 'M':
                        if (TryParseDouble(valueStr, out double mValue))
                        {
                            hasMCommand = true;
                            mCommand = ParseMCommand((int)mValue);
                            HandleMCommand(mCommand.Value, ref laserPower);
                        }
                        break;

                    case 'Y':
                        if (TryParseDouble(valueStr, out double yValue))
                        {
                            y = yValue;
                            hasMovement = true;
                        }
                        break;

                    case 'C':
                        if (TryParseDouble(valueStr, out double cValue))
                        {
                            c = cValue;
                            hasMovement = true;
                        }
                        break;

                    case 'Z':
                        if (TryParseDouble(valueStr, out double zValue))
                        {
                            z = zValue;
                            hasMovement = true;
                        }
                        break;

                    case 'F':
                        if (TryParseDouble(valueStr, out double fValue))
                        {
                            f = fValue;
                            _currentFeedRate = f;
                        }
                        break;

                    case 'S':
                        if (TryParseDouble(valueStr, out double sValue))
                        {
                            laserPower = sValue;
                            _currentLaserPower = laserPower;
                        }
                        break;

                    case 'P':
                        // Dwell parameter, ignore for now
                        break;
                }
            }

            // Process the parsed line
            if (hasMovement || (hasGCommand && IsMovementCommand(currentLineGCommand)))
            {
                // Create movement command
                bool isRapid = currentLineGCommand == GCodeCommandType.G00;
                bool laserOn = !isRapid && _isLaserOn;

                var command = new GCodeCommand3D(y, c, z, f, laserOn, isRapid, currentLineGCommand, rawLine);
                command.LaserPower = laserOn ? _currentLaserPower : 0;
                Commands.Add(command);

                // Update current position
                _currentY = y;
                _currentC = c;
                _currentZ = z;

                System.Diagnostics.Debug.WriteLine($"Added movement: Y={y:F3} C={c:F3} Z={z:F3} Laser={laserOn} Power={command.LaserPower}");
            }
            else if (hasMCommand && mCommand.HasValue)
            {
                // Handle non-movement M commands
                if (mCommand == GCodeCommandType.M02 || mCommand == GCodeCommandType.M30)
                {
                    // Program end
                    var endCommand = new GCodeCommand3D(_currentY, _currentC, _currentZ,
                        _currentFeedRate, false, false, mCommand.Value, rawLine);
                    Commands.Add(endCommand);
                    return; // Stop parsing
                }
            }
            // Standalone F commands are handled by updating _currentFeedRate
        }

        private GCodeCommandType ParseGCommand(int gCode)
        {
            switch (gCode)
            {
                case 0:
                    _modalGCommand = GCodeCommandType.G00;
                    return GCodeCommandType.G00;
                case 1:
                    _modalGCommand = GCodeCommandType.G01;
                    return GCodeCommandType.G01;
                case 2:
                    _modalGCommand = GCodeCommandType.G02;
                    return GCodeCommandType.G02;
                case 3:
                    _modalGCommand = GCodeCommandType.G03;
                    return GCodeCommandType.G03;
                case 4:
                    return GCodeCommandType.G04; // Dwell
                case 90:
                    return GCodeCommandType.G90; // Absolute positioning
                case 91:
                    return GCodeCommandType.G91; // Relative positioning
                default:
                    System.Diagnostics.Debug.WriteLine($"Unknown G-code: G{gCode}, using current modal");
                    return _modalGCommand;
            }
        }

        private GCodeCommandType? ParseMCommand(int mCode)
        {
            switch (mCode)
            {
                case 2:
                    return GCodeCommandType.M02;
                case 3:
                    return GCodeCommandType.M03;
                case 4:
                    return GCodeCommandType.M04;
                case 5:
                    return GCodeCommandType.M05;
                case 30:
                    return GCodeCommandType.M30;
                default:
                    System.Diagnostics.Debug.WriteLine($"Unknown M-code: M{mCode}");
                    return null;
            }
        }

        private void HandleMCommand(GCodeCommandType mCommand, ref double laserPower)
        {
            switch (mCommand)
            {
                case GCodeCommandType.M03:
                case GCodeCommandType.M04:
                    _isLaserOn = true;
                    if (laserPower > 0)
                        _currentLaserPower = laserPower;
                    break;
                case GCodeCommandType.M05:
                    _isLaserOn = false;
                    _currentLaserPower = 0;
                    break;
            }
        }

        private bool IsMovementCommand(GCodeCommandType command)
        {
            return command == GCodeCommandType.G00 ||
                   command == GCodeCommandType.G01 ||
                   command == GCodeCommandType.G02 ||
                   command == GCodeCommandType.G03;
        }

        private bool TryParseDouble(string value, out double result)
        {
            return double.TryParse(value, NumberStyles.Any, CultureInfo.InvariantCulture, out result);
        }
    }
}