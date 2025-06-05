using System;

namespace TubeLaserCAM.UI.Helpers
{
    public static class AngleHelper
    {
        /// <summary>
        /// Normalize góc về khoảng [0, 360)
        /// </summary>
        public static double NormalizeAngle(double angle)
        {
            angle = angle % 360;
            if (angle < 0) angle += 360;
            return angle;
        }

        /// <summary>
        /// Tính delta góc ngắn nhất giữa 2 góc (kết quả trong khoảng -180 đến +180)
        /// Dương = CCW (ngược chiều kim đồng hồ), Âm = CW (cùng chiều kim đồng hồ)
        /// </summary>
        public static double ShortestAngleDelta(double fromAngle, double toAngle)
        {
            // Normalize cả 2 góc về [0, 360)
            fromAngle = NormalizeAngle(fromAngle);
            toAngle = NormalizeAngle(toAngle);

            double delta = toAngle - fromAngle;

            // Nếu delta > 180, quay ngược sẽ ngắn hơn
            if (delta > 180)
                delta = delta - 360;
            // Nếu delta < -180, quay xuôi sẽ ngắn hơn
            else if (delta < -180)
                delta = delta + 360;

            return delta;
        }

        /// <summary>
        /// Interpolate giữa 2 góc theo đường ngắn nhất
        /// </summary>
        /// <param name="fromAngle">Góc bắt đầu</param>
        /// <param name="toAngle">Góc kết thúc</param>
        /// <param name="t">Interpolation factor (0.0 to 1.0)</param>
        public static double InterpolateAngle(double fromAngle, double toAngle, double t)
        {
            double delta = ShortestAngleDelta(fromAngle, toAngle);
            double result = fromAngle + delta * t;
            return NormalizeAngle(result);
        }

        /// <summary>
        /// Kiểm tra xem 2 góc có vượt qua vùng giao 0/360 không
        /// </summary>
        public static bool CrossesSeam(double fromAngle, double toAngle)
        {
            fromAngle = NormalizeAngle(fromAngle);
            toAngle = NormalizeAngle(toAngle);
            double rawDelta = toAngle - fromAngle;
            return Math.Abs(rawDelta) > 180;
        }

        /// <summary>
        /// Tính khoảng cách góc (luôn dương)
        /// </summary>
        public static double AngleDistance(double angle1, double angle2)
        {
            return Math.Abs(ShortestAngleDelta(angle1, angle2));
        }
    }
}