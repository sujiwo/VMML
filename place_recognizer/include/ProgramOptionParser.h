/*
 * Parse program options using Boost::ProgramOptions
 */

#ifndef _PROGRAM_OPTION_PARSER_H
#define _PROGRAM_OPTION_PARSER_H 1


#include <string>
#include <vector>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po=boost::program_options;

namespace PrgOpt {


typedef boost::filesystem::path Path;

class ProgramOption {

public:
	ProgramOption():
		myPath(boost::filesystem::current_path()),
		rArgv(nullptr),
		rArgc(0)
	{

	}

	/*
	 * Add an option. When user sets it, optionally its value will be stored to S
	 * T must be specifically expressed if you want type outside of string
	 */
	template<typename T=std::string>
	void addSimpleOptions(const std::string &opt, const std::string &description, T* S=nullptr, bool isRequired=false)
	{
		auto value_segm = boost::program_options::value<T>();
		if (isRequired)
			value_segm->required();
		if (S!=nullptr)
			value_segm->notifier([S](const T &v){
				*S = v;
			});
		else
			value_segm->notifier([](const T&v){});

		_options.add_options()(opt.c_str(), value_segm, description.c_str()
		);
	}

	inline boost::program_options::options_description_easy_init addOptions()
	{ return _options.add_options(); }

	/*
	 * Get values from command line, set to default if not available
	 */
	template<typename tp>
	tp get(const std::string &cf, tp defaultValue) const
	{
		try {
			return _optionValues.at(cf).as<tp>();
		} catch (std::exception &e) { return defaultValue; }
	}

	inline void parseCommandLineArgs(int argc, char *argv[])
	{
		rArgc = argc;
		rArgv = argv;

		try {
			po::store(po::parse_command_line(argc, argv, _options), _optionValues);
			po::notify(_optionValues);
		} catch (po::error &e) {
			std::cerr << "Parameter error: " << e.what() << std::endl;
			showHelp();
		}
		if (_optionValues.count("help")) showHelp();
	}

	inline void showHelp() const
	{
		// XXX: fix me
		std::cout << _options << std::endl;
		exit(1);
	}

private:
	boost::program_options::options_description _options;
	boost::program_options::variables_map _optionValues;
	Path myPath;

	// Raw arguments
	int rArgc;
	char **rArgv;

};



}	// namespace PrgOpt


#endif // _PROGRAM_OPTION_PARSER_H
