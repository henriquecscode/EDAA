# Import necessary packages
import csv
import enum
import random
import os
import webbrowser 
import folium
from jinja2 import Template
from folium import plugins
from folium.utilities import normalize
import pandas
import sys
import hashlib

class AirportType(enum.Enum):
    ORIGIN = 1
    DESTINATION = 2
    MIDDLE = 3

m = folium.Map(location=[40.0150, -105.2705], prefer_canvas=True)

BASE_DIR = "./data/solutions/"
solution_file = sys.argv[1]
filepath = BASE_DIR + solution_file
edges = []
with open(os.path.expanduser(filepath), "r") as f:
    number_of_edges = int(f.readline())
    for _ in range(number_of_edges):
        line = f.readline().split(",")
        line = [value.strip() for value in line]
        edges.append(line)

airports_ids = [(int(edge[0]), int(edge[1]), edge[4]) for edge in edges]

airports_file = os.path.expanduser('./data/airports_actually_used.csv')
airports = pandas.read_csv(filepath_or_buffer=airports_file, sep=',')

## Start: green
## End: dark red
## Middle: cadet blue
def plotAirport(point, type):
    '''input: series that contains a numeric named latitude and a numeric named longitude
    this function creates a CircleMarker and adds it to your this_map'''
    popup  = folium.Popup(point['name'], max_width=600, max_height=600)

    if (type == AirportType.ORIGIN):
        folium.vector_layers.Marker(location=[point['lat'], point['lng']], tooltip=point['airport_id'], popup = popup, icon=folium.Icon(icon='plane', color='green')).add_to(m)
    elif (type == AirportType.DESTINATION):
        folium.vector_layers.Marker(location=[point['lat'], point['lng']], tooltip=point['airport_id'], popup = popup, icon=folium.Icon(icon='plane', color='darkred')).add_to(m)
    else:
        folium.vector_layers.Marker(location=[point['lat'], point['lng']], tooltip=point['airport_id'], popup = popup, icon=folium.Icon(icon='plane', color='cadetblue')).add_to(m)

def get_carrier_color(carrier):
    hashed = hashlib.sha256(carrier.encode('utf-8')).hexdigest()
    hex_value_color = hashed[0:6] #hi, it's me. I'm the problem, it's me
    return "#" + hex_value_color

## Edges: light gray booooooo
def plotFlight(edge,color="black"):
    popup = folium.Popup(edge, max_width=600, max_height=600)
    line = folium.vector_layers.PolyLine(locations=edge, tooltip=edge[0], popup = popup, color=color, weight=2.5, opacity=0.8)
    line.add_to(m)


for i, (origin, dest, carrier) in enumerate(airports_ids):
    origin_airport = airports.loc[airports['airport_id'] == origin].reset_index()
    dest_airport = airports.loc[airports['airport_id'] == dest].reset_index()
    edge = [[origin_airport['lat'][0], origin_airport['lng'][0]], [dest_airport['lat'][0], dest_airport['lng'][0]]]
    color = get_carrier_color(carrier)
    print(color)
    plotFlight(edge, color)

    if (i == 0): 
        plotAirport(origin_airport, AirportType.ORIGIN)
        plotAirport(dest_airport, AirportType.MIDDLE)
    elif (i == len(airports_ids) - 1):
        plotAirport(origin_airport, AirportType.MIDDLE)
        plotAirport(dest_airport, AirportType.DESTINATION)
    else:
        plotAirport(origin_airport, AirportType.MIDDLE)
        plotAirport(dest_airport, AirportType.MIDDLE)


# DayofMonth,DayOfWeek,Carrier,OriginAirportID,DestAirportID,DepDelay,ArrDelay,distance,flight_time
# airports = pandas.read_csv(filepath_or_buffer='data/airports_actually_used.csv', sep=',')
# airports = pandas.DataFrame(data=airports)[['airport_id', 'lat', 'lng']]

#Set the zoom to the maximum possible
m.fit_bounds(m.get_bounds())
folium.LayerControl().add_to(m)
plugins.MiniMap().add_to(m)

# Display the map
m.save("map.html")
webbrowser.open("map.html")

