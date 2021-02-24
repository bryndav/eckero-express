#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const double pi = 3.141592653589793;
const long earth_radius = 6371000;

typedef struct position
{
    double latitude;
    double longitude;
    struct position* next;
}Pos;

// Create positions around Rotholmen
Pos start_pos = {59.30843, 17.9785, NULL};
Pos dest_1 = {59.31121, 17.9765, NULL};
Pos dest_2 = {59.31227, 17.9789, NULL};
Pos dest_3 = {59.31351, 17.97629, NULL};
Pos dest_4 = {59.31218, 17.97218, NULL};
Pos dest_5 = {59.30842, 17.97846, NULL};

double calcDistance(Pos, Pos);
double calcBearing(Pos, Pos);

int
main()
{
    double distance, bearing;
    double total_distance = 0.0;
    double next_distance = 0.0;
    double next_bearing = 0.0;
    Pos* current_pos = &start_pos;

    // Link positions
    start_pos.next = &dest_1;
    dest_1.next = &dest_2;
    dest_2.next = &dest_3;
    dest_3.next = &dest_4;
    dest_4.next = &dest_5;

    while(current_pos->next != NULL) {
        next_distance = calcDistance(*current_pos, *current_pos->next);
        next_bearing = calcBearing(*current_pos, *current_pos->next);

        printf("Distance to next position: %f m\n", next_distance);
        printf("Bearing to next position: %f deg\n\n\n", next_bearing);

        total_distance = total_distance + next_distance;
        current_pos = current_pos->next;
    }

    printf("Total distance between points: %f m \n", total_distance);

    distance = calcDistance(start_pos, dest_1);
    bearing = calcBearing(start_pos, dest_1);
}

double
calcDistance(Pos start_pos,
             Pos destination)
{
    // Accepts signed decimal degrees without compass direction
    // Negative values indicates west/south (e.g 40.7486, -73.9864)
    double lat_start_pos_rad, lat_dest_rad, delta_lat_rad, delta_lon_rad;
    double a, c, distance;

    lat_start_pos_rad = start_pos.latitude * (pi/180.0);
    lat_dest_rad = destination.latitude * (pi/180.0);
    delta_lat_rad = (start_pos.latitude - destination.latitude) * (pi/180.0);
    delta_lon_rad = (start_pos.longitude - destination.longitude) * (pi/180.0);

    a = sin(delta_lat_rad/2.0) * sin(delta_lat_rad/2.0) +
        cos(lat_start_pos_rad) * cos(lat_dest_rad) *
        sin(delta_lon_rad/2.0) * sin(delta_lon_rad/2.0);

    c = 2 * atan2(sqrt(a), sqrt(1-a));

    distance = earth_radius * c;

    return distance;
}

double
calcBearing(Pos start_pos,
            Pos destination)
{
    // Accepts signed decimal degrees without compass direction
    // Negative values indicates west/south (e.g 40.7486, -73.9864)
    double lat_start_pos_rad, lat_dest_rad, delta_lon_rad;
    double y, x, bearing;

    lat_start_pos_rad = start_pos.latitude * (pi/180.0);
    lat_dest_rad = destination.latitude * (pi/180.0);
    delta_lon_rad = (start_pos.longitude - destination.longitude) * (pi/180.0);

    y = sin(delta_lon_rad) * cos(lat_dest_rad);
    x = cos(lat_start_pos_rad) * sin(lat_dest_rad) -
        sin(lat_start_pos_rad) * cos(lat_dest_rad) * cos(delta_lon_rad);

    bearing = atan2(y, x);
    bearing = bearing * (180.0/pi);
    bearing = (((int)bearing + 360) % 360);

    if (bearing < 0){
        bearing = bearing * -1;
    }else{
        bearing = 360 - bearing;
    }

    return bearing;
}
