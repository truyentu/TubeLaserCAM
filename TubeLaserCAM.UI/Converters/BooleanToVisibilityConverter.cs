// TubeLaserCAM.UI/Converters/BooleanToVisibilityConverter.cs
using System;
using System.Globalization;
using System.Windows;
using System.Windows.Data;

namespace TubeLaserCAM.UI.Converters // Đảm bảo namespace này đúng
{
    [ValueConversion(typeof(bool), typeof(Visibility))]
    public class BooleanToVisibilityConverter : IValueConverter
    {
        /// <summary>
        /// Gets or sets the Visibility value to use when the boolean is true.
        /// Default is Visibility.Visible.
        /// </summary>
        public Visibility TrueVisibility { get; set; } = Visibility.Visible;

        /// <summary>
        /// Gets or sets the Visibility value to use when the boolean is false.
        /// Default is Visibility.Collapsed.
        /// </summary>
        public Visibility FalseVisibility { get; set; } = Visibility.Collapsed;

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            bool boolValue = false;
            if (value is bool b)
            {
                boolValue = b;
            }
            else if (value is bool?)
            {
                bool? nullableBool = (bool?)value;
                boolValue = nullableBool.HasValue && nullableBool.Value;
            }

            // Kiểm tra ConverterParameter để đảo ngược logic nếu cần
            // Ví dụ: Parameter="Reverse" hoặc Parameter="Invert"
            if (parameter is string stringParameter &&
                (stringParameter.Equals("Reverse", StringComparison.OrdinalIgnoreCase) ||
                 stringParameter.Equals("Invert", StringComparison.OrdinalIgnoreCase) ||
                 stringParameter.Equals("Not", StringComparison.OrdinalIgnoreCase)))
            {
                boolValue = !boolValue;
            }

            return boolValue ? TrueVisibility : FalseVisibility;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value is Visibility visibilityValue)
            {
                bool baseResult = visibilityValue == TrueVisibility;

                if (parameter is string stringParameter &&
                    (stringParameter.Equals("Reverse", StringComparison.OrdinalIgnoreCase) ||
                     stringParameter.Equals("Invert", StringComparison.OrdinalIgnoreCase) ||
                     stringParameter.Equals("Not", StringComparison.OrdinalIgnoreCase)))
                {
                    return !baseResult;
                }
                return baseResult;
            }
            return false; // Hoặc DependencyProperty.UnsetValue
        }
    }
}