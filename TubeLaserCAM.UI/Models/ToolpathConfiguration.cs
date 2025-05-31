using System;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace TubeLaserCAM.UI.Models
{
    [Serializable]
    public class ToolpathConfiguration
    {
        public string Name { get; set; }
        public DateTime CreatedDate { get; set; }
        public string FilePath { get; set; }
        public List<SavedEdgeInfo> Edges { get; set; }
        public CylinderData CylinderInfo { get; set; }

        public ToolpathConfiguration()
        {
            Edges = new List<SavedEdgeInfo>();
            CreatedDate = DateTime.Now;
        }
    }

    [Serializable]
    public class SavedEdgeInfo
    {
        public int EdgeId { get; set; }
        public string EdgeType { get; set; }
        public double Length { get; set; }
        public int Order { get; set; }
    }
}