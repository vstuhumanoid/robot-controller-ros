//
// Created by garrus on 14.04.18.
//

#include "EasyLocker.h"

EasyLocker::EasyLocker() :
    mutex_(),
    variable_()
{
    is_notified_ = false;
}

void EasyLocker::Notify()
{
    SetFlag();
    variable_.notify_one();
}

void EasyLocker::NotifyAll()
{
    SetFlag();
    variable_.notify_all();
}

void EasyLocker::SetFlag()
{
    {
        std::lock_guard<std::mutex> guard(mutex_);
        is_notified_++;
    }
}


void EasyLocker::Wait()
{
    std::unique_lock<std::mutex> lock(mutex_);
    variable_.wait(lock, [this]{return this->is_notified_; });
}

