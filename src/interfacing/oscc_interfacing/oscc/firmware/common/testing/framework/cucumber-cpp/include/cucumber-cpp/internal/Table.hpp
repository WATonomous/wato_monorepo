// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CUKE_TABLE_HPP_
#define CUKE_TABLE_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace cucumber
{
namespace internal
{

class Table
{
private:
  typedef std::vector<std::string> basic_type;

public:
  typedef std::map<std::string, std::string> hash_row_type;
  typedef basic_type columns_type;
  typedef basic_type row_type;
  typedef std::vector<hash_row_type> hashes_type;

  /**
     * @brief addColumn
     * @param column
     *
     * @throws std::runtime_error
     */
  void addColumn(const std::string column);

  /**
     * @brief addRow
     * @param row
     *
     * @throws std::range_error
     * @throws std::runtime_error
     */
  void addRow(const row_type & row);
  const hashes_type & hashes() const;

private:
  hash_row_type buildHashRow(const row_type & row);

  columns_type columns;
  hashes_type rows;
};

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_TABLE_HPP_ */
