// TubeLaserCAM.UI/MainWindow.xaml.cs
using System.Windows;
using TubeLaserCAM.UI.ViewModels;

namespace TubeLaserCAM.UI
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DataContext = new MainViewModel();
        }
    }
}