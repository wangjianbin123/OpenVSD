#pragma once

// Observer command target

#include <vector>

class InertiaElem;

class Observer : public CObject
{
public:
	Observer();
	Observer(int index,CString mname,int type);
	Observer(const Observer &src);
	Observer& operator=(const Observer &src);

	int getIndex(void) const { return m_index;}
	CString getName(void) const { return m_name;}
	int getType(void) const { return m_type;}

	virtual Observer* clone(void) const { return new Observer(*this); }
	virtual void IntPntResPushBackprocess(InertiaElem &ie) { }
	virtual void save(void) { }

	virtual ~Observer();

public:
	int m_index;
	CString m_name;
	int m_type;
};


