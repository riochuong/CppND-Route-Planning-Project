#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;
const float DIFF = 0.00001;
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

void PromptForInput(float *start_x, float *start_y, float *end_x, float *end_y) {
    int x0;
    int x1;
    int y0;
    int y1;
    while(true) {
        std::cout << "Please input start x position in the range 0-100: ";
        if (std::cin >> x0 && x0 >= 0 && x0 <= 100) {
            *start_x = x0;
            break;
        }
        std::cin.clear();
        std::cout << "Invalid input value " <<  std::endl;
    }
    while(true) {
        std::cout << "Please input start y position in the range 0-100: ";
        if (std::cin >> y0 && y0 >= 0 && y0 <= 100) {
            *start_y = y0;
            break;
        }
        std::cin.clear();
        std::cout << "Invalid input value !!" << std::endl;
    }
    while(true) {
        std::cout << "Please input end x position in the range 0-100: ";
        if (std::cin >> x1 && x1 >= 0 && x1 <= 100) {
            *end_x = x1;
            break;
        }
        std::cin.clear();
        std::cout << "Invalid input value !!" << std::endl;
    }
    while(true) {
         std::cout << "Please input end y position in the range 0-100: ";
        if (std::cin >> y1 && y1 >= 0 && y1 <= 100) {
            *end_y = y1;
            break;
        }
        std::cin.clear();
        std::cout << "Invalid input value !!" << std::endl;
    }
}

int main(int argc, const char **argv)
{    
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl; 
        return -1;   
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // prompt to get user input
    
    PromptForInput(&start_x, &start_y, &end_x, &end_y);

    // Build Model.
    RouteModel model{osm_data};

    // Perform search and render results.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();
    
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
