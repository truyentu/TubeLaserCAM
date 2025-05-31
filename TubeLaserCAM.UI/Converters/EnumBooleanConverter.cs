using System;
using System.Globalization;
using System.Windows.Data;

namespace TubeLaserCAM.UI.Converters
{
    public class EnumBooleanConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (parameter is string parameterString && value != null)
            {
                return parameterString.Equals(value.ToString(), StringComparison.OrdinalIgnoreCase);
            }
            return false;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (parameter is string parameterString && (bool)value)
            {
                return Enum.Parse(targetType, parameterString);
            }
            return Binding.DoNothing;
        }
    }
}