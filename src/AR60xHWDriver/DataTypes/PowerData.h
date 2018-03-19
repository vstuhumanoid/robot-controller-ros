#ifndef POWERDATA_H
#define POWERDATA_H


class PowerData
{
public:
    PowerData();

    enum PowerSupplies
    {
        Supply12V,
        Supply6V1,
        Supply6V2,
        Supply8V1,
        Supply8V2,
        Supply48V
    };
};

#endif // POWERDATA_H
