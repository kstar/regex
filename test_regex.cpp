/***************************************************************************
                            test_regex.cpp  -
                             -------------------
    begin                : Thu 29 Dec 2016 09:37:10 PST
    copyright            : (c) 2016 by Akarsh Simha
    email                : akarsh.simha@kdemail.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


/* Project Includes */
#include "regex.h"

/* STL Includes */
#include <iostream>

int main() {
    std::string regex;
    std::cerr << "Enter regex: ";
    std::getline( std::cin, regex );
    Regex re( regex );
                 std::cout << re.visualizeDfa();
    std::string str;
    do {
        std::cerr << "Enter string (enter \"done\" to quit): ";
        std::getline( std::cin, str );
        if ( str == "done" )
            break;
        std::cerr << "Result: " << std::string( re.dfaMatch( str ) ? "matches!" : "does not match." ) << std::endl;
    } while ( str != "done" );
}
