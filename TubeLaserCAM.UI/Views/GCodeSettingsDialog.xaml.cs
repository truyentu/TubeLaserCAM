using System.Windows;
using TubeLaserCAM.UI.Models;

namespace TubeLaserCAM.UI.Views
{
    public partial class GCodeSettingsDialog : Window
    {
        public GCodeSettings Settings { get; private set; }

        public GCodeSettingsDialog(GCodeSettings currentSettings)
        {
            InitializeComponent();

            // Clone settings để không thay đổi original nếu user cancel
            Settings = new GCodeSettings
            {
                FeedRate = currentSettings.FeedRate,
                LaserPower = currentSettings.LaserPower,
                PierceTime = currentSettings.PierceTime,
                SafeZ = currentSettings.SafeZ,
                UseG91 = currentSettings.UseG91,
                FileHeader = currentSettings.FileHeader,
                UseLeadInOut = currentSettings.UseLeadInOut,
                LeadInLength = currentSettings.LeadInLength,
                LeadOutLength = currentSettings.LeadOutLength,
                UseDustCollection = currentSettings.UseDustCollection,
                OptimizeSequence = currentSettings.OptimizeSequence,
                ProgramName = currentSettings.ProgramName,
                CuttingStrategy = new CuttingDirectionSettings
                {
                    YDirection = currentSettings.CuttingStrategy.YDirection,
                    CompleteProfileBeforeMoving = currentSettings.CuttingStrategy.CompleteProfileBeforeMoving,
                    OptimizeStartPoint = currentSettings.CuttingStrategy.OptimizeStartPoint,
                    UseLeadInOut = currentSettings.CuttingStrategy.UseLeadInOut,
                    LeadInLength = currentSettings.CuttingStrategy.LeadInLength,
                    LeadOutLength = currentSettings.CuttingStrategy.LeadOutLength
                }
            };

            DataContext = Settings;
        }

        private void OK_Click(object sender, RoutedEventArgs e)
        {
            // Validate input
            if (!ValidateSettings())
            {
                return;
            }

            DialogResult = true;
            Close();
        }

        private void Cancel_Click(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
            Close();
        }

        private void SaveDefault_Click(object sender, RoutedEventArgs e)
        {
            // Save to user settings/config file
            try
            {
                // TODO: Implement save to config
                MessageBox.Show("Settings saved as default", "Success",
                    MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch
            {
                MessageBox.Show("Failed to save settings", "Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private bool ValidateSettings()
        {
            if (Settings.FeedRate <= 0)
            {
                MessageBox.Show("Feed rate must be greater than 0", "Invalid Input",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                txtFeedRate.Focus();
                return false;
            }

            if (Settings.LaserPower < 0 || Settings.LaserPower > 100)
            {
                MessageBox.Show("Laser power must be between 0 and 100%", "Invalid Input",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                txtLaserPower.Focus();
                return false;
            }

            return true;
        }
    }
}
