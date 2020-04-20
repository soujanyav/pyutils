#####################################################################################################
#### Author: Soujanya ####
#### Aim: To identify the covered area for each session that is detected 
#### Method: df_session is storing lat, long readings for each session detected.
######       We need to consolidate the locations and utilize scipy's ConvexHull algorithm
######       to compute first minimum bounding polygon and find its area. If possible, plot it may be
#####################################################################################################
###### Additional reading material: may be - we should convert lat, long to cartesian coordinates. 
###### Link here: https://www.sisense.com/blog/polygon-area-from-latitude-and-longitude-using-sql/
###### Area algorithms (though in C++): http://geomalgorithms.com/a01-_area.html

###### Google Maps Javascript library also has computearea function: 
###### https://stackoverflow.com/questions/33785129/how-to-calculate-an-area-based-on-the-set-of-latitude-and-longitude-values-using

###### C# algorithm area of polygon
"""
private static double CalculatePolygonArea(IList<MapPoint> coordinates)
{
    double area = 0;

    if (coordinates.Count > 2)
    {
        for (var i = 0; i < coordinates.Count - 1; i++)
        {
            MapPoint p1 = coordinates[i];
            MapPoint p2 = coordinates[i + 1];
            area += ConvertToRadian(p2.Longitude - p1.Longitude) * (2 + Math.Sin(ConvertToRadian(p1.Latitude)) + Math.Sin(ConvertToRadian(p2.Latitude)));
        }

        area = area * 6378137 * 6378137 / 2;
    }

    return Math.Abs(area);
}

private static double ConvertToRadian(double input)
{
    return input * Math.PI / 180;
}
"""

import json, math
from scipy.spatial import ConvexHull, convex_hull_plot_2d, Delaunay
import matplotlib.pyplot as plt
import numpy as np

def getConvexHullArea(df_session, size):
    locations = []
    for i in range(size):
        jsonObj = json.loads(df_session.iloc[i,2])
        (lat, long) = (float(jsonObj["lat"]), float(jsonObj["long"]))
        (x, y) = ( long * 3.14159 * 6371 / 180 * math.cos(math.radians(lat)), lat * 3.14159 * 6371 / 180)
        locations.append([x, y])
    locations = np.array(locations)
    hull = ConvexHull(locations)
    plt.plot(locations[:,0], locations[:,1], 'o')
    plt.plot(locations[hull.vertices,0], locations[hull.vertices,1], 'r--', lw=2)
    plt.plot(locations[hull.vertices[0],0], locations[hull.vertices[0],1], 'ro')
    plt.show()
    
    print("Area of this polygon: ", hull.area)
    return hull.area

def getDelaunayTriangulationArea(df_session, size):
    locations = []
    for i in range(size):
        jsonObj = json.loads(df_session.iloc[i,2])
        (lat, long) = (float(jsonObj["lat"]), float(jsonObj["long"]))
        (x, y) = ( long * 3.14159 * 6371 / 180 * math.cos(math.radians(lat)), lat * 3.14159 * 6371 / 180)
        locations.append([x, y])
    locations = np.array(locations)
    tri = Delaunay(locations)
    plt.triplot(locations[:,0], locations[:,1], tri.simplices.copy())
    plt.plot(locations[:,0], locations[:,1], 'o')
    plt.show()
    
    area= 0
    print("Area of this polygon: ", area)
    return area
