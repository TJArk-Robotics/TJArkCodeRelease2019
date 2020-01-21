/**
 * @file BehaviorOption.h
 *
 * This file declares classes and macros to declare behavior options.
 * This is similar to declaring modules. However, behavior options
 * inherit from interfaces
 *
 * @author Arne Hasselbring (some parts borrowed from Src/Tools/Module.h)
 */

#pragma once

#include <vector>

class BehaviorOptionInterface
{
public:
    /** Virtual destructor for polymorphism. */
    virtual ~BehaviorOptionInterface() = default;

    /** Executes the behavior. */
    virtual void execute() = 0;

protected:
    /** Modifies the parameters of the behavior. */
    virtual void modifyParameters() = 0;

    /** Is called every frame before any option is executed. */
    virtual void preProcess()
    {
    }

    /** Is called every frame after all options are executed. */
    virtual void postProcess()
    {
    }

    friend class BehaviorOptionRegistry;
};

class BehaviorOptionWithValue : public virtual BehaviorOptionInterface
{
public:
    /**
   * Returns how good it would be to execute this behavior now.
   * @return A value that indicates how good it would be to execute this behavior now.
   */
    virtual float value() const = 0;
};

class BehaviorOptionWithConditions : public virtual BehaviorOptionInterface
{
public:
    /**
   * Returns whether the behavior can be executed if it has not been executed in the previous frame.
   * @return Whether the behavior...
   */
    virtual bool preconditions() const = 0;

    /**
   * Returns whether the behavior can be executed if it has been executed in the previous frame.
   * @return Whether the behavior...
   */
    virtual bool invariants() const = 0;
};

class BehaviorOptionBase
{
private:
    static BehaviorOptionBase *first;               /**< The head of the list of all behavior options. */
    BehaviorOptionBase *next;                       /**< The next entry in the list of all behavior options. */
    const char *name;                               /**< The name of the behavior option that can be created by this instance. */
    std::vector<const char *> (*getRequirements)(); /**< A function that returns the requirements of this behavior option. */

protected:
    /**
   * Abstract method to create an instance of a behavior option.
   * @return The address of the instance created.
   */
    virtual BehaviorOptionInterface *createNew() = 0;

public:
    /**
   * Constructor.
   * @param name The name of the behavior option that can be created by this instance.
   * @param getRequirements The function that returns the requirements.
   */
    BehaviorOptionBase(const char *name, std::vector<const char *> (*getRequirements)()) : next(first), name(name), getRequirements(getRequirements)
    {
        first = this;
    }
    /**
   * Adds the requirements of all behavior options to the info of a module
   * @param info The info to which the requirements are added.
   */
    // static void addToModuleInfo(std::vector<ModuleBase::Info> &info);

    friend class BehaviorOptionRegistry;
};

template <typename O>
class BehaviorOption : public BehaviorOptionBase
{
private:
    /**
   * Creates a new instance of this option.
   * @return A pointer to the new instance.
   */
    BehaviorOptionInterface *createNew() override
    {
        return new O;
    }

public:
    /**
   * Constructor.
   * @param name The name of the behavior option.
   * @param getRequirements The function that returns the requirements.
   */
    BehaviorOption(const char *name, std::vector<const char *> (*getRequirements)()) : BehaviorOptionBase(name, getRequirements)
    {
    }
};