/*!
  \file        artoolkit_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/10

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\namespace artoolkit_utils
Some useful functions for dealing with "ar_pose/ARMarker" messages from
ARToolkit in ROS (http://wiki.ros.org/ar_pose)
 */

#ifndef ARTOOLKIT_UTILS_H
#define ARTOOLKIT_UTILS_H

#include <ar_pose/ARMarker.h>
#include <ar_pose/ARMarkers.h>
// AD
#include "vision_utils/map_direct_search.h"
#include "vision_utils/map_reverse_search.h"
#include "vision_utils/find_and_replace.h"
#include "vision_utils/retrieve_file_split.h"
#include "vision_utils/cast_from_string.h"
#include "vision_utils/map_to_string.h"

namespace artoolkit_utils {

/*!
 * \class Id2PatternName converts patterns IDs (integers)
 * into their full name
 */
class Id2PatternName {
public:
  typedef int Id;
  typedef std::string PatternName;

  //////////////////////////////////////////////////////////////////////////////

  //! empty ctor
  Id2PatternName() {}

  //! ctor and parse a file
  Id2PatternName(const std::string & filepath) {
    parse(filepath);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! parse file \arg filepath to populate the map and \return true if parsing success
  bool parse(const std::string & filepath) {
    _map.clear();
    // read file
    std::vector<std::string> lines;
    vision_utils::retrieve_file_split(filepath, lines);
    // remove comments, empty spaces, etc.
    for (unsigned int line_idx = 0; line_idx < lines.size(); ++line_idx) {
      vision_utils::remove_trailing_spaces(lines[line_idx]);
      vision_utils::remove_beginning_spaces(lines[line_idx]);
      // remove comments
      if (lines[line_idx].size() > 0 && lines[line_idx][0] == '#') // comment
        lines[line_idx] = "";
      // remove the line if it is empty
      if (lines[line_idx].size() == 0) {
        lines.erase(lines.begin() + line_idx);
        --line_idx;
      }
    } // end loop line_idx

    if (lines.size() == 0) {
      ROS_INFO("parse('%s'): file not existing, full of comments or empty", filepath.c_str());
      return true;
    }

    // get number of blocks
    int nlines = lines.size(), nblocks = vision_utils::cast_from_string<int>(lines[0]);
    if (nblocks < 0 || nblocks > 100) {
      ROS_WARN("parse('%s'): incorrect nb of blocks:%i\n", filepath.c_str(), nblocks);
      return false;
    }
    int expected_nlines = 4 * nblocks + 1;
    if (nlines != expected_nlines) {
      ROS_WARN("parse('%s'): %i blocks, expected %i lines, got %i lines\n",
               filepath.c_str(), nblocks, expected_nlines, nlines);
      return false;
    }

    // parse each block
    for (int block_idx = 0; block_idx < nblocks; ++block_idx) {
      PatternName name = lines[1 + 4 * block_idx];
      _map.insert(std::pair<Id, PatternName>(block_idx, name));
    } // end loop block_idx

    return true;
  } // end parse()

  //////////////////////////////////////////////////////////////////////////////

  //! \return the number of keys (id, pattern) in the map
  inline int nkeys() const { return _map.size(); }

  //////////////////////////////////////////////////////////////////////////////

  //! \return a string representation of the map
  inline std::string to_string() const { return vision_utils::map_to_string(_map); }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Get the pattern name, given a specific ID.
   * \param id
   * \param pattern
   *    the pattern name matching \arg id if it is in the map, undefined otherwise
   * \return
   *    true if lookup success
   */
  inline bool direct_lookup(const Id & id,
                     PatternName & pattern) const {
    return vision_utils::direct_search(_map, id, pattern);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Get the ID, given a specific pattern name.
   * \param pattern
   * \param id
   *    the ID matching \arg pattern if it is in the map, undefined otherwise
   * \return
   *    true if lookup success
   */
  inline bool reverse_lookup(const PatternName & pattern,
                      Id & id) const {
    return vision_utils::reverse_search(_map, pattern, id);
  }


protected:
  std::map<Id, PatternName> _map;
}; // end class Id2PatternName

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // end namespace artoolkit_utils

#endif // ARTOOLKIT_UTILS_H
