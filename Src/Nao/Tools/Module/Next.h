#pragma once

template <typename T>
class Next
{
public:
    void setNext(const T &next)
    {
        updated = true;
        this->next = next;
    }
    bool hasNext() const {return updated;}

    const T &getNext() const
    {
        const_cast<Next<T>*>(this)->updated = false;
        return next;
    }
private:
    T next;
    bool updated = false;
};