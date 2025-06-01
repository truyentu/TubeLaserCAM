// TubeLaserCAM.UI/Models/GCodeParser.cs
using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using System.Linq;

namespace TubeLaserCAM.UI.Models
{
    public class GCodeParser
    {
        public class GCodeMove
        {
            public double Y { get; set; }
            public double C { get; set; }
            public double Z { get; set; }
            public bool IsRapid { get; set; }  // G0 vs G1
            public bool LaserOn { get; set; }
            public double LaserPower { get; set; }
            public int LineNumber { get; set; }
            public string OriginalLine { get; set; }
            public MoveType Type { get; set; }

            public enum MoveType
            {
                Rapid,          // G0
                Cut,            // G1 with laser on
                Pierce,         // M3 + G4
                LaserOff,       // M5
                ProgramStart,   // First position
                ProgramEnd      // Last position
            }
        }

        public class ParseResult
        {
            public List<GCodeMove> Moves { get; set; } = new List<GCodeMove>();
            public double TotalCutLength { get; set; }
            public double TotalRapidLength { get; set; }
            public int PierceCount { get; set; }
            public double CylinderRadius { get; set; }
            public double CylinderLength { get; set; }
            public string Warnings { get; set; }
        }

        private double currentY = 0;
        private double currentC = 0;
        private double currentZ = 0;
        private bool laserOn = false;
        private double laserPower = 0;

        public ParseResult ParseGCode(string gcode)
        {
            var result = new ParseResult();
            var lines = gcode.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            // Parse header untuk info cylinder
            ParseHeader(lines, result);

            for (int i = 0; i < lines.Length; i++)
            {
                var line = lines[i].Trim();

                // Skip empty lines and pure comments
                if (string.IsNullOrEmpty(line) || line.StartsWith(";"))
                {
                    // Extract cylinder info from comments if available
                    if (line.Contains("Cylinder:"))
                    {
                        ExtractCylinderInfo(line, result);
                    }
                    continue;
                }

                // Parse the line
                var move = ParseLine(line, i);
                if (move != null)
                {
                    result.Moves.Add(move);
                }
            }

            // Calculate statistics
            CalculateStatistics(result);

            // Mark first and last moves
            if (result.Moves.Count > 0)
            {
                result.Moves.First().Type = GCodeMove.MoveType.ProgramStart;
                result.Moves.Last().Type = GCodeMove.MoveType.ProgramEnd;
            }

            return result;
        }

        private GCodeMove ParseLine(string line, int lineNumber)
        {
            GCodeMove move = null;

            // Remove inline comments
            var commentIndex = line.IndexOf(';');
            if (commentIndex >= 0)
                line = line.Substring(0, commentIndex).Trim();

            // G0 - Rapid move
            if (line.StartsWith("G0"))
            {
                move = new GCodeMove
                {
                    IsRapid = true,
                    LaserOn = laserOn,
                    LaserPower = laserPower,
                    LineNumber = lineNumber,
                    OriginalLine = line,
                    Type = GCodeMove.MoveType.Rapid,
                    Y = currentY,
                    C = currentC,
                    Z = currentZ
                };

                ParseCoordinates(line, move);
                UpdateCurrentPosition(move);
            }
            // G1 - Feed move
            else if (line.StartsWith("G1"))
            {
                move = new GCodeMove
                {
                    IsRapid = false,
                    LaserOn = laserOn,
                    LaserPower = laserPower,
                    LineNumber = lineNumber,
                    OriginalLine = line,
                    Type = laserOn ? GCodeMove.MoveType.Cut : GCodeMove.MoveType.Rapid,
                    Y = currentY,
                    C = currentC,
                    Z = currentZ
                };

                ParseCoordinates(line, move);
                UpdateCurrentPosition(move);
            }
            // M3 - Laser on
            else if (line.StartsWith("M3"))
            {
                laserOn = true;
                var match = Regex.Match(line, @"S(\d+\.?\d*)");
                if (match.Success)
                {
                    laserPower = double.Parse(match.Groups[1].Value);
                }

                move = new GCodeMove
                {
                    LaserOn = true,
                    LaserPower = laserPower,
                    LineNumber = lineNumber,
                    OriginalLine = line,
                    Type = GCodeMove.MoveType.Pierce,
                    Y = currentY,
                    C = currentC,
                    Z = currentZ
                };
            }
            // M5 - Laser off
            else if (line.StartsWith("M5"))
            {
                laserOn = false;
                laserPower = 0;

                move = new GCodeMove
                {
                    LaserOn = false,
                    LineNumber = lineNumber,
                    OriginalLine = line,
                    Type = GCodeMove.MoveType.LaserOff,
                    Y = currentY,
                    C = currentC,
                    Z = currentZ
                };
            }

            return move;
        }

        private void ParseCoordinates(string line, GCodeMove move)
        {
            // Parse Y
            var match = Regex.Match(line, @"Y(-?\d+\.?\d*)");
            if (match.Success)
            {
                move.Y = double.Parse(match.Groups[1].Value);
            }

            // Parse C
            match = Regex.Match(line, @"C(-?\d+\.?\d*)");
            if (match.Success)
            {
                move.C = double.Parse(match.Groups[1].Value);
            }

            // Parse Z
            match = Regex.Match(line, @"Z(-?\d+\.?\d*)");
            if (match.Success)
            {
                move.Z = double.Parse(match.Groups[1].Value);
            }
        }

        private void UpdateCurrentPosition(GCodeMove move)
        {
            currentY = move.Y;
            currentC = move.C;
            currentZ = move.Z;
        }

        private void ExtractCylinderInfo(string line, ParseResult result)
        {
            // Example: ; Cylinder: R=50.00, L=200.00
            var rMatch = Regex.Match(line, @"R=(\d+\.?\d*)");
            var lMatch = Regex.Match(line, @"L=(\d+\.?\d*)");

            if (rMatch.Success)
                result.CylinderRadius = double.Parse(rMatch.Groups[1].Value);
            if (lMatch.Success)
                result.CylinderLength = double.Parse(lMatch.Groups[1].Value);
        }

        private void ParseHeader(string[] lines, ParseResult result)
        {
            foreach (var line in lines.Take(20)) // Check first 20 lines
            {
                if (line.Contains("Cylinder:"))
                {
                    ExtractCylinderInfo(line, result);
                    break;
                }
            }
        }

        private void CalculateStatistics(ParseResult result)
        {
            for (int i = 1; i < result.Moves.Count; i++)
            {
                var from = result.Moves[i - 1];
                var to = result.Moves[i];

                // Skip non-movement commands
                if (to.Type == GCodeMove.MoveType.Pierce ||
                    to.Type == GCodeMove.MoveType.LaserOff)
                    continue;

                double distance = CalculateDistance(from, to, result.CylinderRadius);

                if (to.Type == GCodeMove.MoveType.Cut)
                {
                    result.TotalCutLength += distance;
                }
                else if (to.Type == GCodeMove.MoveType.Rapid)
                {
                    result.TotalRapidLength += distance;
                }

                // Count pierces
                if (to.Type == GCodeMove.MoveType.Pierce)
                {
                    result.PierceCount++;
                }
            }
        }

        private double CalculateDistance(GCodeMove from, GCodeMove to, double radius)
        {
            // Distance in Y
            double deltaY = to.Y - from.Y;

            // Arc length for C movement
            double deltaC = to.C - from.C;

            // Handle wrap-around
            if (Math.Abs(deltaC) > 180)
            {
                if (deltaC > 0)
                    deltaC -= 360;
                else
                    deltaC += 360;
            }

            // Convert angle to arc length
            double arcLength = Math.Abs(deltaC) * Math.PI / 180.0 * radius;

            // Total distance
            return Math.Sqrt(deltaY * deltaY + arcLength * arcLength);
        }
    }
}