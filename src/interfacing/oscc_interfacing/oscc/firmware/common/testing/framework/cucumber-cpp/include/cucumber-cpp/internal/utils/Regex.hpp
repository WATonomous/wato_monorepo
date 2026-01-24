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

#ifndef CUKE_REGEX_HPP_
#define CUKE_REGEX_HPP_

#include <vector>

#include <boost/regex.hpp>
#include <boost/shared_ptr.hpp>

namespace cucumber
{
namespace internal
{

struct RegexSubmatch
{
  std::string value;
  int position;
};

class RegexMatch
{
public:
  typedef std::vector<RegexSubmatch> submatches_type;

  virtual ~RegexMatch() {};

  bool matches();
  const submatches_type & getSubmatches();

protected:
  bool regexMatched;
  submatches_type submatches;
};

class FindRegexMatch : public RegexMatch
{
public:
  FindRegexMatch(const boost::regex & regexImpl, const std::string & expression);
};

class FindAllRegexMatch : public RegexMatch
{
public:
  FindAllRegexMatch(const boost::regex & regexImpl, const std::string & expression);
};

class Regex
{
private:
  boost::regex regexImpl;

public:
  Regex(std::string expr);

  boost::shared_ptr<RegexMatch> find(const std::string & expression) const;
  boost::shared_ptr<RegexMatch> findAll(const std::string & expression) const;

  std::string str() const;
};

}  // namespace internal
}  // namespace cucumber

#endif /* CUKE_REGEX_HPP_ */
