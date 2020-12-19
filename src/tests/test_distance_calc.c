#include <stdio.h>
#include <stdlib.h>
#include <math.h>

const double pi = 3.141592653589793;
const int earth_radius = 6371000;

typedef struct position
{
    double latitude;
    double longitude;
    struct position* next;
}Pos;

// Create positions around Rotholmen
Pos home = {59.308815, 17.982730, NULL};
Pos dest_1 = {59.311320, 17.977138, NULL};
Pos dest_2 = {59.312224, 17.978804, NULL};
Pos dest_3 = {59.313011, 17.976174, NULL};
Pos dest_4 = {59.312269, 17.973334, NULL};
Pos dest_5 = {59.310837, 17.974859, NULL};
Pos dest_6 = {59.310837, 17.974859, NULL};
Pos dest_7 = {59.308815, 17.982730, NULL};

double calcDistance(Pos, Pos);
double calcBearing(Pos, Pos);

int
main()
{
    double distance, bearing;
    double total_distance = 0.0;
    Pos* current_pos = &home;

    // Link positions
    home.next = &dest_1;
    dest_1.next = &dest_2;
    dest_2.next = &dest_3;
    dest_3.next = &dest_4;
    dest_4.next = &dest_5;
    dest_5.next = &dest_6;
    dest_6.next = &dest_7;

    while(current_pos->next != NULL) {
        total_distance = total_distance + calcDistance(*current_pos, *current_pos->next);
        current_pos = current_pos->next;
    }

    printf("Total distance between points: %f m \n", total_distance);

    distance = calcDistance(home, dest_1);
    bearing = calcBearing(home, dest_1);

    printf("Distance to target: %f m\n", distance);
    printf("Bearing to target: %f deg\n", bearing);
}

double
calcDistance(Pos home,
             Pos destination)
{
    // Accepts signed decimal degrees without compass direction
    // Negative values indicates west/south (e.g 40.7486, -73.9864)
    double lat_home_rad, lat_dest_rad;
    double delta_lat_rad, delta_lon_rad;
    double a, c, distance;

    lat_home_rad = home.latitude * (pi/180.0);
    lat_dest_rad = destination.latitude * (pi/180.0);
    delta_lat_rad = (home.latitude - destination.latitude) * (pi/180.0);
    delta_lon_rad = (home.longitude - destination.longitude) * (pi/180.0);

    a = sin(delta_lat_rad/2.0) * sin(delta_lat_rad/2.0) +
        cos(lat_home_rad) * cos(lat_dest_rad) *
        sin(delta_lon_rad/2.0) * sin(delta_lon_rad/2.0);

    c = 2 * atan2(sqrt(a), sqrt(1-a));

    distance = earth_radius * c;

    return distance;
}

double
calcBearing(Pos home,
            Pos destination)
{
    // Accepts signed decimal degrees without compass direction
    // Negative values indicates west/south (e.g 40.7486, -73.9864)
    double lat_home_rad, lat_dest_rad;
    double delta_lon_rad;
    double y, x, bearing;

    lat_home_rad = home.latitude * (pi/180.0);
    lat_dest_rad = destination.latitude * (pi/180.0);
    delta_lon_rad = (home.longitude - destination.longitude) * (pi/180.0);

    y = sin(delta_lon_rad) * cos(lat_dest_rad);
    x = cos(lat_home_rad) * sin(lat_dest_rad) -
        sin(lat_home_rad) * cos(lat_dest_rad) * cos(delta_lon_rad);

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
