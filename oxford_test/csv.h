/*
 * csv.h
 *
 *  Created on: Jul 30, 2018
 *      Author: sujiwo
 */

#ifndef _CSV_H_
#define _CSV_H_

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <string>
#include <vector>
#include <set>
#include <exception>
#include <sstream>
#include <map>


typedef std::vector<std::vector<std::string>>
	StringTableRecords;


inline double cstod(const std::string &s)
{
	std::istringstream iss(s);
	// force using standard locale
	iss.imbue(std::locale("C"));
	double d;
	iss >> d;
	return d;
}


class StringTable
{
public:
	StringTable() {}

	friend StringTable create_table(const std::string &path, const std::set<int> &usingColumns, bool firstColumnIsHeader);

//	template<typename T>
//	T get (int r, int c)
//	{}

//	template<>
//	std::string get<std::string>(int r, int c)
//	{ return recs.at(r).at(c); }
//
//	template<>
//	uint64_t get<uint64_t>(int r, int c)
//	{ return stoul(recs.at(r).at(c)); }

	const std::string &get(int r, int c) const
	{ return recs.at(r).at(c); }

	double getd(int r, int c) const
	{ return cstod(get(r, c)); }

	const std::string &get(int r, const std::string &s) const
	{
		return recs.at(r).at(header.at(s));
	}

	const size_t size() const
	{ return recs.size(); }

	size_t columns() const
	{ return recs.at(0).size(); }

	size_t rows() const
	{ return recs.size(); }

	void setHeader(const std::vector<std::string> &hdr)
	{
		for (int i=0; i<hdr.size(); i++) {
			header.insert(std::make_pair(hdr[i], i));
		}
	}

protected:
	StringTableRecords recs;
	std::map<std::string, int>header;
};


StringTable create_table(const std::string &path, const std::set<int> &usingColumns=std::set<int>(), bool firstColumnIsHeader=false)
{
	std::ifstream fd;
	fd.open(path);
	if (fd.good()==false)
		throw std::runtime_error("Unable to open file "+path);

	StringTable strTbl;
	bool gotFirstRow = false;

	while (fd.eof() == false) {
		std::string curLine;
		std::getline(fd, curLine);
		if (curLine.size()==0)
			break;

		boost::char_separator<char> sep(", ");
		boost::tokenizer<boost::char_separator<char>> stok(curLine, sep);
		std::vector<std::string> vecline;
		for (auto s: stok) {
			vecline.push_back(s);
		}

		if (usingColumns.size()==0) {
			if (firstColumnIsHeader && gotFirstRow==false) {
				strTbl.setHeader(vecline);
			}
			else
				strTbl.recs.push_back(vecline);
		}

		else {
			std::vector<std::string> vl;
			for (auto &c: usingColumns) {
				vl.push_back(vecline.at(c));
			}
			if (firstColumnIsHeader and gotFirstRow==false) {
				strTbl.setHeader(vl);
			}
			else
				strTbl.recs.push_back(vl);
		}

		if (gotFirstRow==false) {
			gotFirstRow = true;
		}
	}

	return strTbl;
}


#endif /* _CSV_H_ */
