using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Windows; // For Point in GCodeCommand (consider removing if not strictly needed for 3D point)
// using System.Windows.Media.Media3D; // Potentially for Point3D if GCodeCommand3D stores it directly

namespace TubeLaserCAM.Models
{
    public class GCodeParser3D
    {
        private double _currentY = 0;
        private double _currentC = 0;
        private double _currentZ = 0; // Assuming Z starts at 0 or a defined safe height
        private double _currentFeedRate = 0;
        private bool _isLaserOn = false;
        private GCodeCommandType _modalGCommand = GCodeCommandType.G01; // Default or last G0/G1/G2/G3

        public List<GCodeCommand3D> Commands { get; private set; }

        public GCodeParser3D()
        {
            Commands = new List<GCodeCommand3D>();
        }

        public List<GCodeCommand3D> Parse(string filePath)
        {
            Commands.Clear();
            _currentY = 0;
            _currentC = 0;
            _currentZ = 0;
            _currentFeedRate = 0;
            _isLaserOn = false;
            _modalGCommand = GCodeCommandType.G01;

            var lines = File.ReadAllLines(filePath);
            ParseLines(lines);
            return Commands;
        }

        public List<GCodeCommand3D> Parse(IEnumerable<string> gcodeLines)
        {
            Commands.Clear();
            _currentY = 0;
            _currentC = 0;
            _currentZ = 0;
            _currentFeedRate = 0;
            _isLaserOn = false;
            _modalGCommand = GCodeCommandType.G01;

            ParseLines(gcodeLines.ToArray());
            return Commands;
        }

        private void ParseLines(string[] lines)
        {
            foreach (var rawLine in lines)
            {
                var line = rawLine.Trim().ToUpper();
                if (string.IsNullOrWhiteSpace(line) || line.StartsWith(";") || line.StartsWith("("))
                    continue;

                // Remove comments if any (e.g. text after a semicolon)
                int commentIndex = line.IndexOf(';');
                if (commentIndex >= 0)
                {
                    line = line.Substring(0, commentIndex).Trim();
                }
                commentIndex = line.IndexOf('(');
                if (commentIndex >= 0)
                {
                    int endCommentIndex = line.IndexOf(')', commentIndex);
                    if (endCommentIndex > commentIndex)
                    {
                        line = line.Remove(commentIndex, endCommentIndex - commentIndex + 1).Trim();
                    }
                }


                double y = _currentY;
                double c = _currentC;
                double z = _currentZ;
                double f = _currentFeedRate;
                bool ySet = false;
                bool cSet = false;
                bool zSet = false;
                bool fSet = false;

                GCodeCommandType currentLineGCommand = _modalGCommand; // Assume modal unless overridden
                bool gCommandOnLine = false;

                var parts = Regex.Split(line, @"(?=[GMXYZFCIJK])").Where(s => !string.IsNullOrWhiteSpace(s)).ToArray();

                foreach (var part in parts)
                {
                    if (part.Length < 2) continue;
                    char commandChar = part[0];
                    string valueStr = part.Substring(1);
                    double value;

                    if (!double.TryParse(valueStr, NumberStyles.Any, CultureInfo.InvariantCulture, out value))
                    {
                        // Handle cases like "G0" or "M3" where value might be implicit or part of the command code itself
                        if (valueStr.Length == 0 && (commandChar == 'G' || commandChar == 'M'))
                        {
                            // This case needs careful handling based on G-code dialect.
                            // For G0, M3, M5 etc. the number is part of the command itself.
                            // Let's assume the Regex split handles "G00" as one part.
                            // This 'if' might be redundant if Regex is perfect.
                        }
                        else
                        {
                            Console.WriteLine($"Warning: Could not parse value for {part} in line: {rawLine}");
                            continue;
                        }
                    }


                    switch (commandChar)
                    {
                        case 'G':
                            currentLineGCommand = (GCodeCommandType)Enum.Parse(typeof(GCodeCommandType), "G" + ((int)value).ToString("00"));
                            _modalGCommand = currentLineGCommand; // Update modal G command
                            gCommandOnLine = true;
                            break;
                        case 'M':
                            var mCommand = (GCodeCommandType)Enum.Parse(typeof(GCodeCommandType), "M" + ((int)value).ToString("00"));
                            if (mCommand == GCodeCommandType.M03 || mCommand == GCodeCommandType.M04)
                                _isLaserOn = true;
                            else if (mCommand == GCodeCommandType.M05)
                                _isLaserOn = false;
                            else if (mCommand == GCodeCommandType.M02 || mCommand == GCodeCommandType.M30)
                            {
                                Commands.Add(new GCodeCommand3D(_currentY, _currentC, _currentZ, _currentFeedRate, _isLaserOn, false, mCommand, rawLine));
                                return; // Program end
                            }
                            break;
                        case 'Y':
                            y = value;
                            ySet = true;
                            break;
                        case 'C':
                            c = value;
                            cSet = true;
                            break;
                        case 'X': // Assuming X might be used for Y in some configurations, or just to be safe
                            y = value; // Or map X to another axis if needed
                            ySet = true;
                            break;
                        case 'Z':
                            z = value;
                            zSet = true;
                            break;
                        case 'F':
                            f = value;
                            fSet = true;
                            break;
                            // I, J, K for arcs - not implemented in this basic 3D version yet
                    }
                }

                bool isRapid = currentLineGCommand == GCodeCommandType.G00;
                bool laserShouldBeOnForMove = !isRapid && _isLaserOn && (currentLineGCommand == GCodeCommandType.G01 || currentLineGCommand == GCodeCommandType.G02 || currentLineGCommand == GCodeCommandType.G03);

                if (ySet || cSet || zSet || gCommandOnLine) // Only add command if there's a move or explicit G command
                {
                    // If it's a G00 move, laser is off for the move itself, regardless of M3/M4 state
                    bool actualLaserStateForThisCommand = isRapid ? false : laserShouldBeOnForMove;

                    Commands.Add(new GCodeCommand3D(y, c, z, f, actualLaserStateForThisCommand, isRapid, currentLineGCommand, rawLine));
                    _currentY = y;
                    _currentC = c;
                    _currentZ = z;
                    if (fSet) _currentFeedRate = f;
                }
            }
        }

        // Helper to extract value for a given G-Code character (e.g., X, Y, Z, F)
        // This specific helper might be less needed with the Regex approach but can be adapted.
        private double? GetValue(string line, char address)
        {
            var match = Regex.Match(line, $"{address}([-+]?[0-9]*\\.?[0-9]+)");
            if (match.Success)
            {
                return double.Parse(match.Groups[1].Value, CultureInfo.InvariantCulture);
            }
            return null;
        }
    }
}