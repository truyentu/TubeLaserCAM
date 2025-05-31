using System;
using System.Globalization;
using System.Windows.Data;
using GeometryWrapper;

namespace TubeLaserCAM.UI.Converters
{
    public class FilterParameters
    {
        public ManagedEdgeInfo.EdgeType? SelectedType { get; set; }
        public double? MinLength { get; set; }
        public double? MaxLength { get; set; }
    }

    public class FilterParametersConverter : IMultiValueConverter
    {
        public object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
        {
            var filterParams = new FilterParameters();

            // Parse edge type
            if (values[0] is string typeStr && typeStr != "All Types")
            {
                if (Enum.TryParse<ManagedEdgeInfo.EdgeType>(typeStr, out var type))
                    filterParams.SelectedType = type;
            }

            // Parse min length
            if (values[1] is string minStr && double.TryParse(minStr, out var min))
                filterParams.MinLength = min;

            // Parse max length
            if (values[2] is string maxStr && double.TryParse(maxStr, out var max))
                filterParams.MaxLength = max;

            return filterParams;
        }

        public object[] ConvertBack(object value, Type[] targetTypes, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}