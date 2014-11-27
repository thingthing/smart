
//          Copyright Joakim Karlsson & Kim Gräsman 2010-2013.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef IGLOO_MINIMALPROGRESSTESTLISTENER_H
#define IGLOO_MINIMALPROGRESSTESTLISTENER_H

namespace igloo {

  class MinimalProgressTestListener : public TestListener
  {
    public:
      virtual void TestRunStarting() {}
      virtual void TestRunEnded(const TestResults&) 
      {
        std::cout << std::endl;
      }

      virtual void ContextRunStarting(const ContextBase& context)
      {
	std::string name = context.Name();
	std::string const	&hasParent = context.GetAttribute("hasParent");

	std::replace(name.begin(), name.end(), '_', ' ');
	std::cout << DEFAULT_OUTPUT_COLOR << hasParent << "When " <<  name << std::endl;
      }
      virtual void ContextRunEnded(const ContextBase& context)
      {
	const std::string	&hasChild = context.GetAttribute("hasChild");

	if (hasChild.empty())
	  std::cout << std::endl;
      }
      virtual void SpecRunStarting(const ContextBase& context, const std::string& name)
      {
	std::string cpy = name;
	std::string const	&hasParent = context.GetAttribute("hasParent");

	std::replace(cpy.begin(), cpy.end(), '_', ' ');
	std::cout << DEFAULT_OUTPUT_COLOR << hasParent << "\t Then " << cpy << std::endl;;
      }
      virtual void SpecSucceeded(const ContextBase& context, const std::string& )
      {
	std::string const	&hasParent = context.GetAttribute("hasParent");

	std::cout << PASSED_OUTPUT_COLOR << hasParent << "\t SUCCESS" << std::endl;
      }

      virtual void SpecFailed(const ContextBase& context, const std::string& )
      {
	std::string const	&hasParent = context.GetAttribute("hasParent");

	std::cout << FAILED_OUTPUT_COLOR << hasParent << "\t FAILURE" << std::endl;
      }
  };

}
#endif
