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

#ifndef CUKE_TAG_HPP_
#define CUKE_TAG_HPP_

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
using boost::shared_ptr;

#include "../utils/Regex.hpp"

namespace cucumber
{
namespace internal
{

class TagExpression
{
public:
  typedef std::vector<std::string> tag_list;

  virtual ~TagExpression()
  {}

  virtual bool matches(const tag_list & tags) = 0;
};

class OrTagExpression : public TagExpression
{
public:
  OrTagExpression(const std::string & csvTagNotation);
  bool matches(const tag_list & tags);

private:
  bool orTagMatchesTagList(const std::string & currentOrTag, const tag_list & tags);

  tag_list orTags;

  static Regex & csvTagNotationRegex();
};

class AndTagExpression : public TagExpression
{
public:
  AndTagExpression(const std::string & csvTagNotation);
  bool matches(const tag_list & tags);

private:
  typedef std::list<shared_ptr<OrTagExpression> > or_expressions_type;
  or_expressions_type orExpressions;

  static Regex & csvTagNotationRegex();
};

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_TAG_HPP_ */
