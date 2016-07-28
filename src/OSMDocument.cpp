/***************************************************************************
 *   Copyright (C) 2008 by Daniel Wendt                                    *
 *   gentoo.murray@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <vector>
#include <map>
#include <utility>
#include <string>
#include "./OSMDocument.h"
#include "./Configuration.h"
#include "./Node.h"
#include "./Relation.h"
#include "./Way.h"
#include "./math_functions.h"

namespace osm2pgr {

OSMDocument::OSMDocument(Configuration &config) : m_rConfig(config) {
}

OSMDocument::~OSMDocument() {
    ez_mapdelete(m_Nodes);
    ez_vectordelete(m_Ways);
    ez_vectordelete(m_Relations);
    ez_vectordelete(m_SplittedWays);
}
void OSMDocument::AddNode(Node* n) {
    m_Nodes[n->id] = n;
}

void OSMDocument::AddWay(Way* w) {
    m_Ways.push_back(w);
}

void OSMDocument::AddRelation(Relation* r) {
    m_Relations.push_back(r);
}

Node* OSMDocument::FindNode(long long nodeRefId)
const {
    std::map<long long, Node*>::const_iterator  it = m_Nodes.find(nodeRefId);
    return (it != m_Nodes.end()) ? it->second : 0;
}

void OSMDocument::SplitWays() {
    const double MAX_LENGTH = 0.05;

    std::vector<Way*>::const_iterator it(m_Ways.begin());
    std::vector<Way*>::const_iterator last(m_Ways.end());

    //  splitted ways get a new ID
    long long id = 0;

    while (it != last) {
        Way* currentWay = *it++;

        // ITERATE THROUGH THE NODES
        std::vector<Node*>::const_iterator it_node(currentWay->m_NodeRefs.begin());
        std::vector<Node*>::const_iterator last_node(currentWay->m_NodeRefs.end());

        Node* backNode = currentWay->m_NodeRefs.back();

        while (it_node != last_node) {
            Node* node = *it_node++;
            Node* prevNode = node;
            Node* nextNode = 0;

            Way* splitted_way = new Way(++id, currentWay->visible,
                currentWay->osm_id,
                currentWay->maxspeed_forward,
                currentWay->maxspeed_backward);
            splitted_way->name = currentWay->name;
            splitted_way->type = currentWay->type;
            splitted_way->clss = currentWay->clss;
            splitted_way->oneWayType = currentWay->oneWayType;

            std::map<std::string, std::string>::iterator it_tag(currentWay->m_Tags.begin());
            std::map<std::string, std::string>::iterator last_tag(currentWay->m_Tags.end());
//            std::cout << "Number of tags: " << currentWay->m_Tags.size() << std::endl;
//            std::cout << "First tag: " << currentWay->m_Tags.front()->key << ":" << currentWay->m_Tags.front()->value << std::endl;

            // ITERATE THROUGH THE TAGS

            while (it_tag != last_tag) {
                std::pair<std::string, std::string> pair = *it_tag++;

                splitted_way->AddTag(pair.first, pair.second);
            }

    // GeometryFromText('LINESTRING('||x1||' '||y1||','||x2||' '||y2||')',4326);

            splitted_way->geom = "LINESTRING("+ boost::lexical_cast<std::string>(node->lon) + " " + boost::lexical_cast<std::string>(node->lat);

            splitted_way->AddNodeRef(node);

            // Advance through the way's nodes
            while (it_node != last_node) {

                nextNode = *it_node;
                double length = getLength(prevNode, nextNode);
                if (length < 0) length*=-1;

                // Close the splitted way if we'd be exceeding max length by adding the next node,
                // but only if this is not the 2nd node in the way
                if (prevNode != node && splitted_way->length + length > MAX_LENGTH) {
                    it_node--;
                    splitted_way->geom+= ")";
                    break;
                }

                // Add the next node to the splitted way
                splitted_way->AddNodeRef(nextNode);
                splitted_way->length += length;
                splitted_way->geom+= ", " + boost::lexical_cast<std::string>(nextNode->lon) + " " + boost::lexical_cast<std::string>(nextNode->lat);
                prevNode = nextNode;

                // Close the way if this is a shared node, and start the next one from the last node added
                if (nextNode->numsOfUse > 1) {
                  splitted_way->geom+= ")";
                  break;
                }

                // Close the way if this is the last node, and advance the iterator to stop the outer loop
                if (backNode == nextNode) {
                  it_node++;
                  splitted_way->geom+= ")";
                  break;
                }

                it_node++;
            }

            // Only add the splitted way if this is not just a one-node way
            if (splitted_way->m_NodeRefs.front() != splitted_way->m_NodeRefs.back()) {
                m_SplittedWays.push_back(splitted_way);
            } else {
                delete splitted_way;
                splitted_way = 0;
            }
        }
    }
}  // end SplitWays

}  // end namespace osm2pgr
