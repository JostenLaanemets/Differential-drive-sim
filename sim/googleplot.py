import csv
import folium
import webbrowser

class GPSPlotter:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.map = None  # Initialize the map later once we have coordinates

    def plot_from_csv(self):
        points = []
        with open(self.csv_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                try:
                    latitude = float(row['Latitude'])
                    longitude = float(row['Longitude'])
                    points.append([latitude, longitude])
                except ValueError:
                    print(f"Invalid data in row: {row}")
                    continue

        if points:
            # Initialize the map at the first coordinate if points exist
            self.map = folium.Map(location=points[0], zoom_start=19, tiles=None)

            # Add markers to the map
            #for point in points:
                #folium.Marker(point, popup='GPS Location').add_to(self.map)

            # Create and add the polyline
            polyline = folium.PolyLine(locations=points, color='blue')
            polyline.add_to(self.map)

            # Add the Esri Satellite tile layer
            folium.TileLayer(
                tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                attr='Esri',
                name='Esri Satellite',
                overlay=True
            ).add_to(self.map)

            # Add a layer control to toggle the tile layer
            folium.LayerControl().add_to(self.map)

            # Save the map to an HTML file and open it in the browser
            map_file_path = 'gps_map.html'
            self.map.save(map_file_path)
            webbrowser.open(map_file_path)
        else:
            print("No valid GPS coordinates found in the CSV file.")

def main():
    csv_file = '14-05-2024-13-48.csv'
    gps_plotter = GPSPlotter(csv_file)
    gps_plotter.plot_from_csv()

if __name__ == '__main__':
    main()