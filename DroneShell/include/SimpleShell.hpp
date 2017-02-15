// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_SimpleShell_hpp
#define air_SimpleShell_hpp

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <functional>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include "common/common_utils/Utils.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
//better handle below dependency
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON


STRICT_MODE_OFF
#include "linenoise.hpp"
STRICT_MODE_ON

namespace msr { namespace airlib {


template <class ExecContext>
class SimpleShell {
public:
    struct ShellCommandParameters {
        const std::vector<std::string>& args;
        ExecContext* const context;
        const std::string& command_line;
        SimpleShell<ExecContext>* shell_ptr;
    };

    struct ShellCommandSwitch {
        std::string default_value;
        std::string name;
        std::string help;
        std::string value; // current value in executing command.

        ShellCommandSwitch() {}
        ShellCommandSwitch(const std::string& theName, const std::string& defaultValue, const std::string& helpText)
            : default_value(defaultValue), name(theName), help(helpText) 
        {
        }
        ShellCommandSwitch(const ShellCommandSwitch& other):
            default_value(other.default_value), name(other.name), help(other.help), value(other.value)
        {
        }
        void operator=(const ShellCommandSwitch& other) {
            name = other.name;
            default_value = other.default_value;
            help = other.help;
            value = other.value;
        }
    };

    class ShellCommand {
    private:    
        const std::string name_;
        const std::string help_;
        std::unordered_map<std::string, ShellCommandSwitch> switches_;
    public:

        ShellCommand(std::string name, std::string help)
        : name_(name), help_(help)
        {
        }

        void addSwitch(const ShellCommandSwitch& s) {
            std::string lower = boost::algorithm::to_lower_copy(s.name);
            switches_[lower] = s;
        }

        const std::string& getName() const { return name_; }

        const std::string& getHelp() const { return help_; }

        ShellCommandSwitch& getSwitch(const std::string& name) {
            std::string lower = boost::algorithm::to_lower_copy(name);

            if (switches_.find(lower) != switches_.end()){
                ShellCommandSwitch& result = switches_.at(lower);
                return result;
            }                        
            throw std::invalid_argument(common_utils::Utils::stringf("switch %s is not defined on this command", name.c_str()));                        
        }

        int getSwitchInt(const std::string& name)
        {
            return std::stoi(getSwitch(name).value);
        }

        const std::vector<std::string> getSwitches() const { 
            std::vector<std::string> result;
            for (auto kv : switches_) {
                result.push_back(kv.first);
            }
            return result; 
        }

        virtual bool execute(const ShellCommandParameters&) { return false; };
    private:
        ShellCommand(){}
        ShellCommand(ShellCommand& other){}
    };


    typedef std::function<std::string (const ShellCommandParameters& params, std::string script_filepath)> BeforeScriptStartCallback;
    BeforeScriptStartCallback _beforeScriptStartCallback;
    void beforeScriptStartCallback(BeforeScriptStartCallback value) {
        _beforeScriptStartCallback = value;
    }
    typedef std::function<bool (const ShellCommandParameters& params, std::string script_filepath)> AfterScriptEndCallback;
    AfterScriptEndCallback _afterScriptEndCallback;
    void afterScriptEndCallback(AfterScriptEndCallback value) {
        _afterScriptEndCallback = value;
    }
    typedef std::function<std::string (const ShellCommandParameters& params, std::string command_line)> BeforeScriptCommandStartCallback;
    BeforeScriptCommandStartCallback _beforeScriptCommandStartCallback;
    void beforeScriptCommandStartCallback(BeforeScriptCommandStartCallback value) {
        _beforeScriptCommandStartCallback = value;
    }
    typedef std::function<bool (const ShellCommandParameters& params, std::string command_line, bool commandReturnValue)> AfterScriptCommandEndCallback;
    AfterScriptCommandEndCallback _afterScriptCommandEndCallback;
    void afterScriptCommandEndCallback(AfterScriptCommandEndCallback value) {
        _afterScriptCommandEndCallback = value;
    }

    typedef std::function<void (const ShellCommandParameters& params, std::string command_line)> BeforeCommandStartCallback;
    BeforeCommandStartCallback _beforeCommandStartCallback;
    void beforeCommandStartCallback(BeforeCommandStartCallback value) {
        _beforeCommandStartCallback = value;
    }
    typedef std::function<bool (const ShellCommandParameters& params, std::string command_line, bool commandReturnValue)> AfterCommandEndCallback;
    AfterCommandEndCallback _afterCommandEndCallback;
    void afterCommandEndCallback(AfterCommandEndCallback value) {
        _afterCommandEndCallback = value;
    }

private:    //typedefs and sub types
    typedef std::vector<std::string> VectorString;
    typedef boost::escaped_list_separator<char> CommandSeparator;
    typedef boost::tokenizer<CommandSeparator> CommandTokenizer;
   
public: //default commands
    bool runScriptCommand(const ShellCommandParameters& params) {
        if (params.args.size() > 1) {
            std::string command_line, filePath = params.args[1];
            if (_beforeScriptStartCallback)
                _beforeScriptStartCallback(params, filePath);
            std::ifstream scriptFile(filePath);
            while(std::getline(scriptFile, command_line)) {
                if (_beforeScriptCommandStartCallback)
                        _beforeScriptCommandStartCallback(params, command_line);
                bool commandReturnValue = execute(command_line, params.context);
                if (_afterScriptCommandEndCallback)
                        commandReturnValue = _afterScriptCommandEndCallback(params, command_line, commandReturnValue);
                if (commandReturnValue)
                    return true;
            }
            if (_afterScriptEndCallback)
                return _afterScriptEndCallback(params, filePath);
            return false;
        }
        else {
            std::cout << "Please specify the script to execute as first argument" << std::endl;
            return false;
        }
    }

    typedef std::function<bool (const ShellCommandParameters&)> SimpleShellCommand;

    class CommentCommand : public ShellCommand {
    public:
        CommentCommand() : ShellCommand("rem", "Comment out the line")
        {
        }

        bool execute(const ShellCommandParameters& params) 
        {
            return false;
        }
    };
    CommentCommand commentCommand;


    class QuitCommand : public ShellCommand {
    public:
        QuitCommand() : ShellCommand("quit", "Exit the shell")
        {
        }

        bool execute(const ShellCommandParameters& params) 
        {
            return true;
        }
    };
    QuitCommand quitCommand;


    class HelpCommand : public ShellCommand {
        SimpleShellCommand helpMethod_;
    public:
        HelpCommand() : ShellCommand("help", "Help on the supported commands or Help [Command] for help on a specific command")
        {
        }
        void bind(SimpleShellCommand helpMethod){
            helpMethod_ = helpMethod;
        }

        bool execute(const ShellCommandParameters& params) 
        {
            return helpMethod_(params);
        }
    };

    HelpCommand helpCommand;

    bool helpMethod(const ShellCommandParameters& params)
    {
        if (params.args.size() == 2) 
        {
            std::string name = boost::algorithm::to_lower_copy(params.args[1]);

            if (command_infos_.find(name) != command_infos_.end()) {
                ShellCommand& command = command_infos_.at(name);

                std::cout << command.getName() << "  " << command.getHelp() << std::endl;

                size_t longest = 0;
                auto switches = command.getSwitches();
                for (const std::string& switchName : switches)
                {
                    size_t len = switchName.size();
                    if (len > longest) longest = len;
                }
                for (const std::string& switchName : switches)
                {
                    ShellCommandSwitch& sw = command.getSwitch(switchName);
                    std::cout << "   " << sw.name;
                    std::cout << std::string(longest + 5 - sw.name.size(), ' ');
                    std::cout << sw.help << std::endl;
                }
                return false;
            } else {
                std::cout << "### command '" << params.args[1] << "' not found" << std::endl;
                std::cout << std::endl;
            }
        }

        size_t longest = 0;
        for(auto kv : command_infos_) {
            size_t len = kv.first.size();
            if (len > longest) longest = len;
        }
        for(auto alias : command_aliases_) {
            size_t len = alias.first.size();
            if (len > longest) longest = len;
        }

        std::vector<std::string> sorted;
        for (auto kv : command_infos_) {
            sorted.push_back(kv.second.getName());
        }
        std::sort(sorted.begin(), sorted.end(), [&](const std::string& p1, const std::string& p2) {
            return p1.compare(p2) < 0;
        });

        for(auto name : sorted) {

            std::cout << name;
            std::string lowerName = boost::algorithm::to_lower_copy(name);
            ShellCommand& cmd = command_infos_.at(lowerName);
            std::cout << std::string(longest + 3 - name.size(), ' ');
            std::cout << cmd.getHelp() << std::endl;
            for(auto alias : command_aliases_) {

                std::string lowerAlias = boost::algorithm::to_lower_copy(alias.second);

                if (lowerAlias == name) {
                    std::cout << " " << alias.first;
                    std::cout << std::string(longest + 3 - alias.first.size(), ' ');
                    std::cout << "same as " <<  name << std::endl;
                }
            }
        }
        return false;
    }


    class RunCommand : public ShellCommand {
        SimpleShellCommand runMethod_;
    public:
        RunCommand() : ShellCommand("run", "Run script specified in file")
        {
        }
        void bind(SimpleShellCommand runMethod){
            runMethod_ = runMethod;
        }

        bool execute(const ShellCommandParameters& params) 
        {
            return runMethod_(params);
        }
    };
    RunCommand runCommand;

private:
    std::unordered_map<std::string, ShellCommand&> command_infos_;
    std::unordered_map<std::string, std::string> command_aliases_;
    int command_history_size_;
    std::string prompt_;
private:
    void commandCompletitionCallBack(const char* editBuffer, VectorString& completions) {
        for(auto kv : command_infos_) {
            if (boost::starts_with(kv.first, editBuffer))
                completions.push_back(kv.first);
        }
    }
public:
    SimpleShell(std::string prompt = "> ")
    {
        runCommand.bind(std::bind(&SimpleShell::runScriptCommand, this, std::placeholders::_1));
        helpCommand.bind(std::bind(&SimpleShell::helpMethod, this, std::placeholders::_1));

        //Add default commands and aliases
        addCommand(quitCommand);
        addCommand(helpCommand);
        addCommand(runCommand);
        addCommand(commentCommand);
                        
        addAlias("q", "quit");
        addAlias("exit", "quit");
        addAlias("?", "help");
        addAlias("#", "rem");
        
        //Setup the shell
        setCommandHistorySize(50);
        setPrompt(prompt);
        
        //Setup the command completition
        linenoise::SetCompletionCallback(std::bind(&SimpleShell::commandCompletitionCallBack, this
        , std::placeholders::_1, std::placeholders::_2));
    }

    // add a reference to a command (this object must remain valid, we do not copy it)
    void addCommand(ShellCommand& command) {
        std::string lower = boost::algorithm::to_lower_copy(command.getName());
        command_infos_.insert({lower, command });
    }
    void removeCommand(std::string command) {
        std::string lower = boost::algorithm::to_lower_copy(command);
        command_infos_.erase(lower);
    }
    void addAlias(std::string alias, std::string command) {
        std::string lower = boost::algorithm::to_lower_copy(alias);
        command_aliases_.insert({lower, command});
    }
    void removeAlias(std::string alias) {
        std::string lower = boost::algorithm::to_lower_copy(alias);
        command_aliases_.erase(lower);
    }

    //getters & setters
    int getCommandHistorySize() const { return command_history_size_; }
    void setCommandHistorySize(int value) { 
        command_history_size_ = value;
        linenoise::SetHistoryMaxLen(50); 
    }

    std::string getPrompt() const { return prompt_; }
    void setPrompt(std::string prompt) { prompt_ = prompt; }
    
    void showMessage(const std::string& message) {
        std::cout << prompt_ << message << std::endl;
    }    

    //template <class ExecContext>
    virtual bool execute(const ShellCommandParameters& params) {
        if (params.args.size() == 0)
            return false;
        std::string command = boost::algorithm::to_lower_copy(params.args[0]);

        ShellCommand* cmd = nullptr;

        if (command_infos_.find(command) != command_infos_.end()) {
            cmd = &command_infos_.at(command);
        } else if (command_aliases_.find(command) != command_aliases_.end()) {
            command = command_aliases_.at(command);
            if (command_infos_.find(command) != command_infos_.end()){
                cmd = &command_infos_.at(command);
            } else {
                std::cout << "Alias '" << params.args[0] << "' referred to unknown command '" << command << "'" << std::endl;
                return false;
            }
        }
        else {
            std::cout << "Command '" << params.args[0] << "' is not supported" << std::endl;
            return false;
        }

        std::vector<std::string> nonSwitchArgs;

        if (cmd != nullptr)
        {
            // parse out any switches (-foo 123) and set them on the command so they are easy to find.
            // and report errors for unexpected switches.
            // first initialize all switches to their  default value.

            for (const std::string& switchName : cmd->getSwitches()){
                ShellCommandSwitch& sw = cmd->getSwitch(switchName);
                sw.value = sw.default_value;
            }

            const std::vector<std::string>& args = params.args;
            int len = static_cast<int>(args.size());
            for(int i = 0; i < len; i++) {
                std::string arg = args.at(i);
                if (!arg.empty() && arg.at(0) == '-') {
                    if (i + 1 < len) {
                        cmd->getSwitch(arg).value = args.at(i + 1);
                        ++i;
                    }
                    else {
                        throw std::invalid_argument(common_utils::Utils::stringf("switch %s is missing a value", arg.c_str()));
                    }
                } else {
                    // pass along any non-switch arguments as positional arguments for the command to process.
                    nonSwitchArgs.push_back(arg);
                }
            }
            
            return cmd->execute(ShellCommandParameters{nonSwitchArgs, params.context, params.command_line, this});
        }
        return false;
    }
    
    bool execute(const std::string& command_line, ExecContext* const context) {
        const CommandSeparator separators('\\', ' ', '\"'); 

        //Setup line editor
        VectorString args;
        
        //parse arguments
        CommandTokenizer tok(command_line, separators);
        args.clear();
        for(auto token = tok.begin(); token != tok.end(); ++token)
            args.push_back(*token);
        
        auto params = ShellCommandParameters{args, context, command_line, this};
        
        try {
            if (_beforeCommandStartCallback)
                _beforeCommandStartCallback(params, command_line);

            bool result = execute(params);

            if (_afterCommandEndCallback)
                result =_afterCommandEndCallback(params, command_line, result);

            return result;
        }
        catch (rpc::rpc_error& rpc_ex) {
            std::cerr << "RPC Error: " << rpc_ex.get_error().as<std::string>() << std::endl;
        }
        catch(const std::exception& ex) {
            std::cerr << "Error occurred: " << ex.what() << std::endl;
        }
        catch(...) {
            // catch any other errors (that we have no information about)
            std::cerr << "Non-standard exception occured" << std::endl;
        }
        
        return false;
    }
    
    //template <class ExecContext>
    bool readLineAndExecute(ExecContext* const context) {
        //get command line
        auto line = linenoise::Readline(prompt_.c_str());
        line = common_utils::Utils::trim(line, ' ');
        if (line.size() > 0) {
            linenoise::AddHistory(line.c_str());
            return execute(line, context);
        }
        return false;
    }    
};

}} //namespace
#endif
